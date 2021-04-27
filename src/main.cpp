#include <iostream>
#include <vector>

#include "ICP.h"

#include "opencascade/STEPCAFControl_Reader.hxx"

std::vector<TopoDS_Shape> ReadShape(const char* filename) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile(filename);
    if (stat) {
        std::cout << "Error reading model" << std::endl;
        throw std::runtime_error("wrong model");
    }
    Standard_Integer num = reader.TransferRoots();

    std::vector<TopoDS_Shape> objects(num);
    for (int i = 0; i < num; i++) {
        objects[i] = reader.OneShape();
    }
    return objects;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout << "MODELS" << std::endl;
        return EXIT_FAILURE;
    }

    auto model = ReadShape(argv[1]);
    auto points = ReadShape(argv[2]);
}
