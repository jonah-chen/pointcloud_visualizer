
#include "mesh.hpp"

int main(int argc, char *argv[])
{
    auto cloud = load_mesh("../tmp/office_1_bpa.ply", true);
    return 0;
}