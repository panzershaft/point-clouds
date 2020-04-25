#include"hello.hpp"

#include<iostream>
using namespace std;
void hello::say_hello(){
    std::cout<<"This is how overidding works"<<endl;
}

int main()
{
    hello::say_hello();
}