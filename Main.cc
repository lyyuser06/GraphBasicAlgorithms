#include "Graph.hh"

Debugger debugger;

int main()
{
    try {
        Graph<int> g1("g1.txt");
        Graph<int> g1_1(g1);
        Graph<int> g1_2 = g1;
        Graph<int> g1_3 = std::move(g1);

        std::cout << g1.getE() << g1.getV() << std::endl;
        debugger.divide_line();

        Graph<int> g2("g2.txt");
        std::cout << g2.getE() << g2.getV() << std::endl;
        debugger.divide_line();
        g2.display("g2_out.dot");
        g2 = g1_2;
        g2.display("g2_out_g1.dot");
        std::cout << g2.getE() << g2.getV() << std::endl;
        debugger.divide_line();

        DiGraph<int> dg3("dg3.txt");
        std::cout << dg3.getE() << dg3.getV() << std::endl;
        debugger.divide_line();
        dg3.display("dg3.dot");

        
        debugger.divide_line();
        std::cout << g1.getE() << g1.getV() << std::endl;
        debugger.divide_line();
    }
    catch(const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    
    return 0;
}