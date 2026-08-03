#define BOOST_TEST_MODULE MoveStreamTest
#include <boost/test/unit_test.hpp>

#include <vector>

#include "../robot_socket.hpp"

#include "protocol.h"

namespace {

// a point whose first joint holds tag, so a test can tell points apart after they have been
// through the stream.
trajectory_point_t tagged_point(double tag) {
    trajectory_point_t pt{};
    pt.positions[0] = tag;
    return pt;
}

std::vector<trajectory_point_t> tagged_points(std::initializer_list<double> tags) {
    std::vector<trajectory_point_t> points;
    points.reserve(tags.size());
    for (const auto tag : tags) {
        points.push_back(tagged_point(tag));
    }
    return points;
}

std::vector<double> tags_of(const std::vector<trajectory_point_t>& points) {
    std::vector<double> tags;
    tags.reserve(points.size());
    for (const auto& point : points) {
        tags.push_back(point.positions[0]);
    }
    return tags;
}

}  // namespace

BOOST_AUTO_TEST_SUITE(move_stream_unary)

// the unary move path wraps its finished trajectory in a stream. that stream starts out closed,
// since the consumer has to know right away that nothing else is coming. otherwise it would think
// the end of the trajectory was the producer falling behind.
BOOST_AUTO_TEST_CASE(seeded_stream_is_born_closed) {
    robot::MoveStream stream{tagged_points({1.0, 2.0, 3.0})};

    BOOST_CHECK(stream.closed());
    BOOST_CHECK(!stream.drained());
    BOOST_CHECK_EQUAL(stream.pending_count(), 3U);
    BOOST_CHECK(!stream.finished());
    BOOST_CHECK(!stream.abort_reason().has_value());
}

BOOST_AUTO_TEST_CASE(seeded_stream_rejects_further_points) {
    robot::MoveStream stream{tagged_points({1.0})};

    BOOST_CHECK(!stream.extend(tagged_points({2.0})));
    BOOST_CHECK_EQUAL(stream.pending_count(), 1U);
}

BOOST_AUTO_TEST_CASE(empty_seeded_stream_is_drained) {
    const robot::MoveStream stream{{}};

    BOOST_CHECK(stream.closed());
    BOOST_CHECK(stream.drained());
    BOOST_CHECK_EQUAL(stream.pending_count(), 0U);
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(move_stream_producer)

BOOST_AUTO_TEST_CASE(extend_then_close_drains) {
    robot::MoveStream stream;

    BOOST_CHECK(!stream.closed());
    BOOST_CHECK(!stream.drained());

    BOOST_CHECK(stream.extend(tagged_points({1.0, 2.0})));
    BOOST_CHECK(stream.extend(tagged_points({3.0})));
    BOOST_CHECK_EQUAL(stream.pending_count(), 3U);

    // closed but not drained yet, the consumer still has points to pick up.
    BOOST_CHECK(stream.close());
    BOOST_CHECK(stream.closed());
    BOOST_CHECK(!stream.drained());

    BOOST_CHECK_EQUAL(stream.take(3).size(), 3U);
    BOOST_CHECK(stream.drained());
}

BOOST_AUTO_TEST_CASE(close_is_end_of_stream_for_the_producer) {
    robot::MoveStream stream;
    BOOST_CHECK(stream.close());
    BOOST_CHECK(!stream.extend(tagged_points({1.0})));
    BOOST_CHECK_EQUAL(stream.pending_count(), 0U);
}

// finish is the consumer saying the move is over. after that everything the producer does returns
// false and changes nothing, which is how a producer still feeding a move finds out it is done,
// either because the goal faulted or because the fsm failed the request before sending it.
BOOST_AUTO_TEST_CASE(finish_retires_the_stream) {
    robot::MoveStream stream;
    BOOST_CHECK(stream.extend(tagged_points({1.0})));

    stream.finish();

    BOOST_CHECK(stream.finished());
    BOOST_CHECK_EQUAL(stream.pending_count(), 0U);
    BOOST_CHECK(!stream.extend(tagged_points({2.0})));
    BOOST_CHECK(!stream.close());
}

BOOST_AUTO_TEST_CASE(finish_is_idempotent) {
    robot::MoveStream stream;
    stream.finish();
    BOOST_CHECK_NO_THROW(stream.finish());
    BOOST_CHECK(stream.finished());
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(move_stream_abort)

BOOST_AUTO_TEST_CASE(abort_closes_and_records_reason) {
    robot::MoveStream stream;
    BOOST_CHECK(stream.extend(tagged_points({1.0})));

    stream.abort("client went away");

    BOOST_CHECK(stream.closed());
    const auto reason = stream.abort_reason();
    BOOST_REQUIRE(reason.has_value());
    BOOST_CHECK_EQUAL(*reason, "client went away");
    // the move is not finished yet, the goal monitor still has to notice and stop the arm.
    BOOST_CHECK(!stream.finished());
}

// the caller should hear about whatever went wrong first, so a later abort, like the teardown path
// aborting again on its way out, must not overwrite it.
BOOST_AUTO_TEST_CASE(first_abort_reason_wins) {
    robot::MoveStream stream;
    stream.abort("first");
    stream.abort("second");

    BOOST_CHECK_EQUAL(*stream.abort_reason(), "first");
}

BOOST_AUTO_TEST_CASE(abort_stops_further_points) {
    robot::MoveStream stream;
    stream.abort("done");
    BOOST_CHECK(!stream.extend(tagged_points({1.0})));
}

// extend and close do nothing once the stream is finished, and abort has to do the same, or we
// could report a move that completed as cancelled.
BOOST_AUTO_TEST_CASE(abort_after_finish_is_a_no_op) {
    robot::MoveStream stream;
    stream.finish();

    stream.abort("too late");

    BOOST_CHECK(!stream.abort_reason().has_value());
}

// finish ends the stream, it does not erase why the move failed.
BOOST_AUTO_TEST_CASE(finish_preserves_an_earlier_abort_reason) {
    robot::MoveStream stream;
    stream.abort("client went away");
    stream.finish();

    const auto reason = stream.abort_reason();
    BOOST_REQUIRE(reason.has_value());
    BOOST_CHECK_EQUAL(*reason, "client went away");
}

BOOST_AUTO_TEST_SUITE_END()

BOOST_AUTO_TEST_SUITE(move_stream_consumer)

BOOST_AUTO_TEST_CASE(take_preserves_order_and_removes) {
    robot::MoveStream stream;
    BOOST_REQUIRE(stream.extend(tagged_points({1.0, 2.0, 3.0, 4.0})));

    const auto first = stream.take(2);
    BOOST_CHECK((tags_of(first) == std::vector<double>{1.0, 2.0}));
    BOOST_CHECK_EQUAL(stream.pending_count(), 2U);

    const auto rest = stream.take(2);
    BOOST_CHECK((tags_of(rest) == std::vector<double>{3.0, 4.0}));
    BOOST_CHECK_EQUAL(stream.pending_count(), 0U);
}

// the goal monitor asks for a full chunk every tick no matter how much the producer has actually
// sent, so getting back fewer points, or none, has to be a normal answer and not an error.
BOOST_AUTO_TEST_CASE(take_more_than_available_returns_what_there_is) {
    robot::MoveStream stream;
    BOOST_REQUIRE(stream.extend(tagged_points({1.0})));

    BOOST_CHECK_EQUAL(stream.take(200).size(), 1U);
    BOOST_CHECK_EQUAL(stream.take(200).size(), 0U);
}

// an open stream that we have taken everything from is not drained, since the producer might still
// send more. only closing it makes an empty buffer mean the trajectory is over.
BOOST_AUTO_TEST_CASE(empty_but_open_is_not_drained) {
    robot::MoveStream stream;
    BOOST_REQUIRE(stream.extend(tagged_points({1.0})));
    BOOST_CHECK_EQUAL(stream.take(1).size(), 1U);

    BOOST_CHECK(!stream.drained());
    BOOST_CHECK(stream.close());
    BOOST_CHECK(stream.drained());
}

BOOST_AUTO_TEST_SUITE_END()
