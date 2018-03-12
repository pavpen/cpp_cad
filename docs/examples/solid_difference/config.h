#include <fstream>
#include <stack>

//
// Operation Log configuration:
//
#define OPERATION_LOG_INIT_FUNCTION_NAMESPACE  output_cylinder_config
#define OPERATION_LOG_INIT_FUNCTION_NAME       operation_log_init

#include <operation_log.h>

namespace output_cylinder_config
{

void operation_log_init(operation_log::DefaultOperationLog &log)
{
    // Filter operation log messages:
    class MessageFilter : public operation_log::RunTimePredicate<const std::stack<operation_log::FunctionInfo>&>
    {
    public:
        bool operator()(const std::stack<operation_log::FunctionInfo>& call_stack)
        {
            return true;
            // const std::string func_name = call_stack.top().get_short_name();

            // return func_name == "advance_prev_parallel_vertex" ||
            //     func_name == "add_vertex";
        }
    };

    // Make sure the message_filter instance isn't destroyied when this
    // function returns:
    static MessageFilter message_filter;

    static std::ofstream output_stream("operation-log.html");
    static operation_log::HtmlFormatter formatter(output_stream, "output_sphere");

    formatter.extra_header_code =
        operation_log::HtmlFormatter::three_js_header_code;

    log.set_message_filter_predicate(message_filter);
    log.set_formatter(formatter);
}

}
