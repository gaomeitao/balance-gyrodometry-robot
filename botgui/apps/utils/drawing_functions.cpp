#include <gtk/gtk.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <typeinfo>
#include <iostream>
#include <vector>

using namespace std;

#include <time.h>
#include "vx/vxo_drawables.h"
#include "vx/gtk/vx_gtk_display_source.h"

// core api
#include "vx/vx_global.h"
#include "vx/vx_layer.h"
#include "vx/vx_world.h"
#include "vx/vx_colors.h"

#define PROJ_HEIGHT 200
#define PROJ_LENGTH 200
#define PROJ_SCALE .05
#define MAX_NUM_PTS 3
#define PATH_SIZE 10

#include <apps/utils/drawing_functions.hpp>
#include <common/pose_trace.hpp>
#include <../lcmtypes/pose_xyt_t.hpp>
#include <../lcmtypes/robot_path_t.hpp>

void draw_grid(vx_buffer_t* buffer)
{
    // draw a grid background
    vx_buffer_add_back(buffer, vxo_chain(vxo_mat_scale(0.10f), vxo_grid()));

    // draw the origin
    vx_buffer_add_back(buffer, vxo_chain(vxo_mat_scale(0.10f), vxo_axes()));
    
}

void draw_robot(const pose_xyt_t& pose, const float* color, vx_buffer_t* buffer)
{
    // define and draw the robot pose
    vx_buffer_add_back(buffer, vxo_chain(vxo_mat_translate3(pose.x, pose.y, 0.0),
                                         vxo_mat_rotate_z(pose.theta),
                                         vxo_mat_scale(0.15f),
                                         vxo_robot(vxo_mesh_style(color))));
}

void draw_pose_trace(const PoseTrace& poses, const float* color, vx_buffer_t* buffer)
{
    // Define and draw the trace
    // Create a resource to hold all the poses
    vx_resc* poseResc = vx_resc_createf(poses.size() * 3);
    float* poseBuf = static_cast<float*>(poseResc->res);
    for(auto& p : poses)
    {
        *poseBuf++ = p.x;
        *poseBuf++ = p.y;
        *poseBuf++ = 0.0;
    }

    vx_object_t* trace = vxo_lines(poseResc,
                                   poses.size(),
                                   GL_LINE_STRIP,
                                   vxo_lines_style(color, 2.0));
    vx_buffer_add_back(buffer, trace);
}

void draw_path(const robot_path_t& path, const float* color, vx_buffer_t* buffer)
{
    // define and draw the path
    if(path.path_length == 0)
    {
        return;
    }

    // Draw the path as line segments between target poses, which are drawn as little robots
    for(auto& pose : path.path)
    {
        vx_buffer_add_back(buffer, vxo_chain(vxo_mat_translate3(pose.x, pose.y, 0.0),
                                             vxo_mat_scale(0.05f),
                                             vxo_box(vxo_mesh_style(color))));
    }

    vx_resc* poseResc = vx_resc_createf(path.path.size() * 3);
    float* poseBuf = static_cast<float*>(poseResc->res);
    for(auto& pose : path.path)
    {
        *poseBuf++ = pose.x;
        *poseBuf++ = pose.y;
        *poseBuf++ = 0.0;
    }

    vx_object_t* trace = vxo_lines(poseResc,
                                   path.path.size(),
                                   GL_LINE_STRIP,
                                   vxo_lines_style(color, 2.0));
    vx_buffer_add_back(buffer, trace);
}
