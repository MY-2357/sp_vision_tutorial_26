#include <fmt/core.h>

int main()
{

    //print
    int a=10, b=20;
    fmt::print("{:d}+{:d}={:d}\npi={:.2f}\n", a, b, a+b, 3.14159);
    //format
    std::string str = fmt::format("{1},{0}!", "World", "Hello");
    fmt::print("{}\n", str);

    return 0;
}