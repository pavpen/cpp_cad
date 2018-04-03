#ifndef _CPP_CAD_OPERATION_LOGGING_TUPLE_H
#define _CPP_CAD_OPERATION_LOGGING_TUPLE_H

#include <tuple>

#include <operation_log.h>


namespace operation_log
{

// A value formatter for the `cpp_cad::Polygon_2` data type.
template <typename... Ts>
class ValueFormatter<std::tuple<Ts...>> : public ValueFormatterI
{
    private:

    const std::tuple<Ts...> value;

    public:

    // Receives the value to format.
    ValueFormatter(const std::tuple<Ts...> value)
    : value(value)
    {}

    // Formats values for plain text logs.
    std::string to_text() override
    {
        std::stringstream res;

        res << "std::tuple( ";
        write_elements<0>(res);
        res << " )";

        return res.str();
    }

    // Formats values for HTML logs.
    std::string to_html() override
    {
        // We can output a Three.js scene, or an SVG element here.
        return HtmlUtils::escape(to_text());
    }

    private:

    template <std::size_t VarI>
    typename std::enable_if<VarI == sizeof...(Ts), void>::type
    write_elements(std::stringstream &res)
    {}

    template <std::size_t VarI>
    typename std::enable_if<VarI < sizeof...(Ts), void>::type
    write_elements(std::stringstream &res)
    {
        if (VarI > 0)
        {
            res << ", ";
        }

        ValueFormatter<typename std::tuple_element<VarI, std::tuple<Ts...>>::type>
            value_formatter(std::get<VarI>(value));

        res << value_formatter.to_text();

        write_elements<VarI + 1>(res);
    }
};

}

#endif // _CPP_CAD_OPERATION_LOGGING_TUPLE_H
