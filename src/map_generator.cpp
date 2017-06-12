#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "map_generator_node");
    
    assert(argc == 2 && "bad input");

    octomap::OcTree tree(0.1);
    for (int x = -40; x < 40; x++) 
    for (int y = -40; y < 40; y++) 
    for (int z = -40; z < 40; z++) 
    {
        double dst = 0;
        dst += sqrt((x + 20) * (x + 20) + y * y + z * z);
        dst += sqrt((x - 20) * (x - 20) + y * y + z * z);
        if (dst > 60) continue;

        octomap::point3d v3(x * 0.05, y * 0.05, z * 0.05);
        tree.updateNode(v3, true);
    }

    tree.writeBinary(argv[1]);

    return 0;
}