#include <aerox_suite/hal/TeraEvo/NewTera.h>

int main(int argc, char **argv){
    NewTera teraBee;
    if( !teraBee.startSensor() ){
        std::cout << "Could not start terabee sensor" << std::endl;
        return -1;
    }
}