#include <fstream>
#include <sstream>
#include <stack>

//
// Operation Log configuration:
//
#define OPERATION_LOG_INIT_FUNCTION_NAMESPACE  output_cylinder_config
#define OPERATION_LOG_INIT_FUNCTION_NAME       operation_log_init

#include <operation_log.h>

#include <cpp_cad.h>

namespace operation_log
{

// A value formatter for the `cpp_cad::Polygon_2` data type.
template <>
class ValueFormatter<cpp_cad::Polygon_2> : public ValueFormatterI
{
    private:

    const cpp_cad::Polygon_2 value;

    public:

    // Receives the value to format.
    ValueFormatter(const cpp_cad::Polygon_2 value)
    : value(value)
    {}

    // Formats values for plain text logs.
    std::string to_text() override
    {
        std::stringstream res;

        res << "Polygon_2 { ";
        typename cpp_cad::Polygon_2::Vertex_const_iterator vit;
        bool is_first = true;

        for (vit = value.vertices_begin(); vit != value.vertices_end(); ++vit)
        {
            if (is_first)
            {
                is_first = false;
            }
            else
            {
                res << ", ";
            }
            res << (*vit);
        }
        res << " }";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        // We can output a Three.js scene, or an SVG element here.
        return HtmlUtils::escape(to_text());
    }
};

}

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

    log.set_message_filter_predicate(message_filter);

    // Output HTML:
    static std::ofstream output_stream("operation-log.html");
    static operation_log::HtmlFormatter formatter(output_stream, "rotate_extrude");

    formatter.extra_header_code =
        operation_log::HtmlFormatter::three_js_header_code;

    log.set_formatter(formatter);
}

}
