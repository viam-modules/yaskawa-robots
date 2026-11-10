#define BOOST_TEST_MODULE TelemetryPathTest
#include <boost/test/unit_test.hpp>

#include <stdexcept>
#include <string>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/config/resource.hpp>

#include "../utils.hpp"

namespace {

viam::sdk::ResourceConfig config_with(viam::sdk::ProtoStruct attributes) {
    return {"arm", "test-arm", "", std::move(attributes), "rdk:component:arm", viam::sdk::Model("test", "test", "test")};
}

}  // namespace

BOOST_AUTO_TEST_SUITE(telemetry_path_tests)

BOOST_AUTO_TEST_CASE(test_extract_trace_id_from_traceparent) {
    BOOST_CHECK_EQUAL(*extract_trace_id_from_traceparent("00-4bf92f3577b34da6a3ce929d0e0e4736-00f067aa0ba902b7-01"),
                      "4bf92f3577b34da6a3ce929d0e0e4736");
    BOOST_CHECK(!extract_trace_id_from_traceparent("01-4bf92f3577b34da6a3ce929d0e0e4736-00f067aa0ba902b7-01"));
    BOOST_CHECK(!extract_trace_id_from_traceparent("00-short-00f067aa0ba902b7-01"));
    BOOST_CHECK(!extract_trace_id_from_traceparent("00-4bf92f3577b34da6a3ce929d0e0e4736-00f067aa0ba902b7"));
    BOOST_CHECK(!extract_trace_id_from_traceparent(""));
}

BOOST_AUTO_TEST_CASE(test_expand_telemetry_path) {
    BOOST_CHECK_EQUAL(expand_telemetry_path("/home/viam/.viam/capture", "tag={trace_id}", "abc").string(),
                      "/home/viam/.viam/capture/tag=abc");
    BOOST_CHECK_EQUAL(expand_telemetry_path("/data", "traces/{trace_id}/{trace_id}", "x").string(), "/data/traces/x/x");
}

BOOST_AUTO_TEST_CASE(test_parse_telemetry_traceid_template) {
    BOOST_CHECK_EQUAL(parse_telemetry_traceid_template(config_with({})), "");
    BOOST_CHECK_EQUAL(parse_telemetry_traceid_template(config_with({{"telemetry_output_path_append_traceid", true}})), "{trace_id}");
    BOOST_CHECK_EQUAL(parse_telemetry_traceid_template(config_with({{"telemetry_output_path_append_traceid", false}})), "");
    BOOST_CHECK_EQUAL(
        parse_telemetry_traceid_template(config_with({{"telemetry_output_path_append_traceid", std::string("tag={trace_id}")}})),
        "tag={trace_id}");
    BOOST_CHECK_THROW(parse_telemetry_traceid_template(config_with({{"telemetry_output_path_append_traceid", std::string("tag=")}})),
                      std::invalid_argument);
    BOOST_CHECK_THROW(parse_telemetry_traceid_template(config_with({{"telemetry_output_path_append_traceid", 1.0}})),
                      std::invalid_argument);
}

BOOST_AUTO_TEST_SUITE_END()
