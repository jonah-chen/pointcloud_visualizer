#include "main.hpp"

int main(int argc, char *argv[])
{
    return _main<m_PointCloud, MeshHUD, MeshRenderer>(argc, argv, load_mesh);
}