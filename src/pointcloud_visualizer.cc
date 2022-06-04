#include "main.hpp"

int main(int argc, char *argv[])
{
    return _main<PointCloud, PointHUD, PointRenderer>(argc, argv, load);
}