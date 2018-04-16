#include <fstream>
#include <sstream>
#include <stack>

//
// Operation Log configuration:
//
#define OPERATION_LOG_INIT_FUNCTION_NAMESPACE  cpp_cad_operation_log
#define OPERATION_LOG_INIT_FUNCTION_NAME       operation_log_init

#include <operation_log.h>

#include <cpp_cad.h>


namespace cpp_cad
{

namespace test
{

namespace config
{

extern std::string unit_test_name;

}

}

}

namespace cpp_cad_operation_log
{

void operation_log_init(operation_log::DefaultOperationLog &log)
{
    // Output HTML:
    static std::ofstream
        output_stream(
            cpp_cad::test::config::unit_test_name + ".operation-log.html");
    static operation_log::HtmlFormatter
        formatter(output_stream, cpp_cad::test::config::unit_test_name);

    formatter.extra_header_code =
        operation_log::HtmlFormatter::three_js_header_code +
        operation_log::HtmlFormatter::math_jax_header_code;

    log.set_formatter(formatter);
}

}
