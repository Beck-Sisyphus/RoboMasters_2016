//swig -python example.i
//gcc -fPIC -c example.c example_wrap.c -I/usr/local/include/python2.7 -lpython2.7

//gcc -shared example.o example_wrap.o -o _example.so
char* fact(int n) {
    if (n <= 1) return "1";
    else return "222";
}
