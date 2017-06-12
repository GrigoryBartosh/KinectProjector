#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/math/Pose6D.h>
#include <iostream>
#include "kinect_projector/KinectFrame.h"
#include "kinect_projector/KinectProjector.h"

const size_t WIDTH_DEFAULT = 640;
const size_t HEIGHT_DEFAULT = 480;
const double TREE_RESOLUTION = 0.1;

void get_tree(octomap::OcTree &tree, const std::string &file)
{
    std::ifstream fin(file);
    assert(fin.is_open() && "opening input file error");
    tree.readBinary(fin);
    fin.close();
}

octomath::Pose6D get_transformation(const std::string &file)
{
    std::ifstream fin(file);
    assert(fin.is_open() && "opening input file error");
    octomath::Pose6D T;
    fin >> T.trans().x() >> T.trans().y() >> T.trans().z() >>
           T.rot().u() >> T.rot().x() >> T.rot().y() >> T.rot().z();
    fin.close();
    return T;
}

void check_sizes(bool f_width, bool f_height, size_t& width, size_t &height)
{
    if (!f_width && !f_height)
    {
        width = WIDTH_DEFAULT;
        height = HEIGHT_DEFAULT;
    }
    else if (!f_width)
    {
        assert(height % 3 == 0 && "bad input");
        width = height / 3 * 4;
    }
    else if (!f_height)
    {
        assert(width % 4 == 0 && "bad input");
        height = width / 4 * 3;
    }
}

void read_input(int argc, char* argv[],
               octomap::OcTree &tree, 
               octomath::Pose6D &Transform,
               size_t &width,
               size_t &height,
               std::string &out)
{
    bool f_tree = false;
    bool f_Transform = false;
    bool f_width = false;
    bool f_height = false;
    bool f_out = false;
    for (int i = 1; i < argc; i++)
    {
        if (!strcmp(argv[i], "-m"))
        {
            assert(i < argc-1 && !f_tree && "bad input");
            i++;
            std::string file(argv[i], strlen(argv[i]));
            get_tree(tree, file);
            f_tree = true;
        }
        else if (!strcmp(argv[i], "-t"))
        {
            assert(i < argc-1 && !f_Transform && "bad input");
            i++;
            std::string file(argv[i], strlen(argv[i]));
            Transform = get_transformation(file);
            f_Transform = true;
        }
        else if (!strcmp(argv[i], "-w"))
        {
            assert(i < argc-1 && !f_width && "bad input");
            i++;
            width = atoi(argv[i]);
            f_width = true;
        }
        else if (!strcmp(argv[i], "-h"))
        {
            assert(i < argc-1 && !f_height && "bad input");
            i++;
            height = atoi(argv[i]);
            f_height = true;
        }
        else if (!strcmp(argv[i], "-o"))
        {
            assert(i < argc-1 && !f_out && "bad input");
            i++;
            out.assign(argv[i], strlen(argv[i]));
            f_out = true;
        }
        else assert(0 && "bad input");
    }

    check_sizes(f_width, f_height, width, height);

    assert(f_tree      && "bad input");
    assert(f_Transform && "bad input");
    assert(f_out       && "bad input");
}

void write_answer(const KinectFrame &projection, const std::string &file)
{
    std::ofstream fout(file);
    assert(fout.is_open() && "opening output file error");
    fout << projection.width() << " " << projection.height() << "\n";
    for (int y = 0; y < projection.height(); y++)
    {
        for (int x = 0; x < projection.width(); x++)
            fout << projection.get(x, y) << " ";
        fout << "\n";
    }
    fout.close();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kinect_projector_node");

    octomap::OcTree tree(TREE_RESOLUTION);
    octomath::Pose6D Transform;
    size_t width, height;
    std::string out;
    read_input(argc, argv, tree, Transform, width, height, out);
    KinectFrame projection(width, height);

    KinectProjector::project(tree, Transform, projection);

    write_answer(projection, out);

	return 0;
}