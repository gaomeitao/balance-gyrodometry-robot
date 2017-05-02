// util includes
#include <apps/botgui/botgui.hpp>
#include <apps/utils/drawing_functions.hpp>
#include <common/timestamp.h>
#include <common/grid_utils.hpp>

// vx/gtk includes
#include <vx/gtk/vx_gtk_display_source.h>
#include <vx/vx_colors.h>
#include <gdk/gdk.h>

// std lib includes
#include <iomanip>
#include <iterator>
#include <sstream>
#include <cassert>
#include <glib.h>
#include <unistd.h>

// channel types
#include <../lcmtypes/balancebot_motorcmd_t.hpp>
#include <../optitrack/optitrack_channels.h>

// TRUE_POSE, ODOMETRY_POSE, 

// Declare gui buttons
void clear_traces_pressed(GtkWidget* button, gpointer gui);
void reset_state_pressed(GtkWidget* button, gpointer gui);

BotGui::BotGui(lcm::LCM* lcmInstance, int argc, char** argv, int widthInPixels,
                int heightInPixels, int framesPerSecond):
                VxGtkWindowBase(argc, argv, widthInPixels, heightInPixels, framesPerSecond),
                havePath_(false), haveTruePose_(false),
                shouldResetStateLabels_(false), shouldClearTraces_(false),
                nextColorIndex_(0), lcmInstance_(lcmInstance)
{
    assert(lcmInstance_);
    
    odometry_.x = odometry_.y = odometry_.theta = 0.0;
    path_.path_length = 0;

    traceColors_.push_back(vx_red);
    traceColors_.push_back(vx_orange);
    traceColors_.push_back(vx_purple);
    traceColors_.push_back(vx_magenta);
    traceColors_.push_back(vx_maroon);
    traceColors_.push_back(vx_forest);
    traceColors_.push_back(vx_navy);
    traceColors_.push_back(vx_olive);
    traceColors_.push_back(vx_plum);
    traceColors_.push_back(vx_teal);
}


void BotGui::clearAllTraces(void)
{
    shouldClearTraces_ = true;
}

// Setup appending paths via mouse clicks
int BotGui::onMouseEvent(vx_layer_t* layer,
                         vx_camera_pos_t* cameraPosition,
                         vx_mouse_event_t* event,
                         Point<float> worldPoint)
{
    // If a Ctrl + Left-click, send a robot_path_t with a single position
    if((event->button_mask & VX_BUTTON1_MASK) && (event->modifiers & VX_CTRL_MASK))
    {
        pose_xyt_t odomPose;
        odomPose.x = odometry_.x;
        odomPose.y = odometry_.y;
        odomPose.theta = odometry_.theta;

        pose_xyt_t target;
        target.x = worldPoint.x;
        target.y = worldPoint.y;
        target.theta = 0.0f;

        // If an odometry trace exists, then we need to transform the Vx reference frame into the odometry frame
        auto odomTraceIt = traces_.find(ODOMETRY_POSE_CHANNEL);
        if(odomTraceIt != traces_.end())
        {
            auto odomToVx = odomTraceIt->second.trace.getFrameTransform();
            // Apply an inverse transform to rotate from Vx to odometry
            double xShifted = worldPoint.x - odomToVx.x;
            double yShifted = worldPoint.y - odomToVx.y;
            target.x = (xShifted * std::cos(-odomToVx.theta)) - (yShifted * std::sin(-odomToVx.theta));
            target.y = (xShifted * std::sin(-odomToVx.theta)) + (yShifted * std::cos(-odomToVx.theta));

            // If we're showing paths and this path is going to be drawn incorrectly, give a warning.
            if(haveTruePose_ && gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(showPathCheck_)))
            {
                std::cout << "WARNING: Optitrack data detected. The Ctrl+Click path will not be displayed correctly.\n";
            }
        }
        // Otherwise, just assume points are in the odometry frame

        std::cout << "Sending controller path to " << target.x << ',' << target.y << " in odometry frame\n";

        robot_path_t path;
        path.path_length = 2;
        path.path.push_back(odomPose);
        path.path.push_back(target);
        lcmInstance_->publish(BALANCEBOT_PATH_CHANNEL, &path);
    }

    std::lock_guard<std::mutex> autoLock(vxLock_);
    mouseWorldCoord_ = worldPoint;
    mouseGridCoord_ = global_position_to_grid_cell(worldPoint, map_);

    return 0;
}

void BotGui::onDisplayStart(vx_display_t* display)
{
    VxGtkWindowBase::onDisplayStart(display);
    lcmInstance_->subscribe(BALANCEBOT_PATH_CHANNEL, &BotGui::handlePath, this);
    lcmInstance_->subscribe(".*_POSE", &BotGui::handlePose, this);  // NOTE: Subscribe to ALL _POSE channels!
}

void BotGui::render(void)
{
    std::lock_guard<std::mutex> autoLock(vxLock_);

    vx_buffer_t* gridBuf = vx_world_get_buffer(world_, "grid");
    draw_grid(gridBuf);
    vx_buffer_swap(gridBuf);
 
    // Draw the current path if requested
    vx_buffer_t* pathBuf = vx_world_get_buffer(world_, "path");
    if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(showPathCheck_)))
    {
        draw_path(path_, vx_forest, pathBuf);
    }
    vx_buffer_swap(pathBuf);

    // Draw all active poses
    vx_buffer_t* poseBuf = vx_world_get_buffer(world_, "poses");
    for(auto& t : traces_)
    {
        Trace& trace = t.second;

        if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(trace.checkbox)))
        {
            draw_pose_trace(trace.trace, trace.color, poseBuf);

            if(!trace.trace.empty())
            {
                draw_robot(trace.trace.back(), trace.color, poseBuf);
            }
        }
    }
    vx_buffer_swap(poseBuf);

    gdk_threads_enter();        // lock the GTK loop to avoid race conditions during modification
    populateNewTraceBoxes();
    destroyTracesIfRequested();
    updateGridStatusBarText();
    gdk_threads_leave();    // no more modifications to the window are happening, so unlock
}

void BotGui::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(vxLock_);
    addPose(*pose, channel);
}

void BotGui::handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* odom)
{
    std::lock_guard<std::mutex> autoLock(vxLock_);

    odometry_ = *odom;

    // Also, save a PoseTrace for the pure odometry output
    pose_xyt_t odomPose;
    odomPose.utime = odom->utime;
    odomPose.x = odom->x;
    odomPose.y = odom->y;
    odomPose.theta = odom->theta;
    addPose(odomPose, channel);
}

void BotGui::handlePath(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const robot_path_t* path)
{
    std::lock_guard<std::mutex> autoLock(vxLock_);
    path_ = *path;
    havePath_ = true;
}

void BotGui::addPose(const pose_xyt_t& pose, const std::string& channel)
{
    auto traceIt = traces_.find(channel);

    // If no trace exists yet for this channel, then create one
    if(traceIt == traces_.end())
    {
        std::cout << "Adding new PoseTrace:" << channel << '\n';
        Trace trace;
        trace.trace.addPose(pose);
        trace.checkbox = gtk_check_button_new_with_label(channel.c_str());
        trace.color = traceColors_[nextColorIndex_];
        nextColorIndex_ = (nextColorIndex_ + 1) % traceColors_.size();

        // If the new trace is TRUE_POSE_CHANNEL, then the frame transform of odometry needs to be updated
        if(channel == TRUE_POSE_CHANNEL)
        {
            haveTruePose_ = true;
            initialTruePose_ = pose;

            // Go through every existing trace and add a new frame transform if it corresponds to odometry
            for(auto& t : traces_)
            {
                if(t.first.find(ODOMETRY_POSE_CHANNEL) != std::string::npos)
                {
                    t.second.trace.setReferencePose(initialTruePose_);
                    std::cout << "Applying TRUE_POSE frame transform to trace on channel: " << t.first << '\n';
                }
            }
        }
        // If a true pose is specified, then we need to change reference frame of odometry if it is being added
        else if(haveTruePose_ && (channel.find(ODOMETRY_POSE_CHANNEL) != std::string::npos))
        {
            trace.trace.setReferencePose(initialTruePose_);
            std::cout << "Applying TRUE_POSE frame transform to trace on channel: " << channel << '\n';
        }

        // Set the color of the text to match the color of the trace
        GtkWidget* checkLabel = gtk_bin_get_child(GTK_BIN(trace.checkbox));

        GdkColor textColor;
        textColor.red = trace.color[0] * 65535;
        textColor.green = trace.color[1] * 65535;
        textColor.blue = trace.color[2] * 65535;

        GdkColor bgColor;
        gdk_color_parse("white", &bgColor);

        gtk_widget_modify_bg(checkLabel, GTK_STATE_NORMAL, &bgColor);
        gtk_widget_modify_bg(checkLabel, GTK_STATE_PRELIGHT, &bgColor);
        gtk_widget_modify_bg(checkLabel, GTK_STATE_ACTIVE, &bgColor);
        gtk_widget_modify_fg(checkLabel, GTK_STATE_NORMAL, &textColor);
        gtk_widget_modify_fg(checkLabel, GTK_STATE_PRELIGHT, &textColor);
        gtk_widget_modify_fg(checkLabel, GTK_STATE_ACTIVE, &textColor);

        traceBoxesToAdd_.push_back(trace.checkbox);
        traces_[channel] = trace;
    }
    // Otherwise, just add a pose to the existing trace
    else
    {
        traceIt->second.trace.addPose(pose);
    }
}

void BotGui::destroyTracesIfRequested(void)
{
    if(shouldClearTraces_)
    {
        // Cleanup all existing trace boxes.
        for(auto& t : traces_)
        {
            gtk_widget_destroy(t.second.checkbox);
        }

        traces_.clear();
        shouldClearTraces_ = false;
        haveTruePose_ = false;
    }
}

void BotGui::populateNewTraceBoxes(void)
{
    if(!traceBoxesToAdd_.empty())
    {
        for(auto& box : traceBoxesToAdd_)
        {
            gtk_box_pack_start(GTK_BOX(tracesBox_), box, FALSE, FALSE, 0);
            gtk_widget_show(box);
            gtk_container_resize_children(GTK_CONTAINER(tracesBox_)); // resize to make sure the full label is visible
        }

        gtk_widget_show(tracesBox_);

        traceBoxesToAdd_.clear();
    }
}

void BotGui::updateGridStatusBarText(void)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << "Global: " << mouseWorldCoord_ << " Cell: " << mouseGridCoord_
        << " Log-odds: " << static_cast<int>(map_.logOdds(mouseGridCoord_.x, mouseGridCoord_.y));

    // For each active trace, write the current pose
    out << "    ";
    for(auto& t : traces_)
    {
        Trace& trace = t.second;
        if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(trace.checkbox)))
        {
            out << t.first << ": (" << trace.trace.back().x << ',' << trace.trace.back().y << ','
                << trace.trace.back().theta << ")   ";
        }
    }

    gtk_statusbar_pop(GTK_STATUSBAR(gridStatusBar_), 0);
    gtk_statusbar_push(GTK_STATUSBAR(gridStatusBar_), 0, out.str().c_str());
}


void BotGui::createGuiLayout(GtkWidget* window, GtkWidget* vxCanvas)
{
    GtkWidget* mainBox = gtk_vbox_new(FALSE, 5);
    gtk_container_add(GTK_CONTAINER(window), mainBox);

    // Create a horizontal pane. Options/command on the right. Vx canvas and sensor data on the left
    GtkWidget* mainPane = gtk_hpaned_new();
//     gtk_container_add(GTK_CONTAINER(window), mainPane);
    gtk_box_pack_start(GTK_BOX(mainBox), mainPane, TRUE, TRUE, 0);

    // Create the Vx and live scoring pane as a vertical box
    GtkWidget* vxBox = gtk_vbox_new(FALSE, 5);
    gtk_paned_pack1(GTK_PANED(mainPane), vxBox, TRUE, FALSE);
    gtk_box_pack_end(GTK_BOX(vxBox), vxCanvas, TRUE, TRUE, 0);
    gtk_widget_show(vxCanvas);    // XXX Show all causes errors!

    // Add the widgets to the right 
    optionsBox_ = gtk_vbox_new(FALSE, 10);
    gtk_paned_pack2(GTK_PANED(mainPane), optionsBox_, FALSE, FALSE);

    ////////  Checkboxes for controlling which data is to be rendered  /////////////
    GtkWidget* dataLabel = gtk_label_new("Data to Show:");
    gtk_box_pack_start(GTK_BOX(optionsBox_), dataLabel, FALSE, TRUE, 0);

    showMapCheck_ = gtk_check_button_new_with_label("Show Map");
    gtk_box_pack_start(GTK_BOX(optionsBox_), showMapCheck_, FALSE, TRUE, 0);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(showMapCheck_), TRUE);

    showPathCheck_ = gtk_check_button_new_with_label("Show Path");
    gtk_box_pack_start(GTK_BOX(optionsBox_), showPathCheck_, FALSE, TRUE, 0);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(showPathCheck_), TRUE);

    GtkWidget* dataSeparator = gtk_hseparator_new();
    gtk_box_pack_start(GTK_BOX(optionsBox_), dataSeparator, FALSE, TRUE, 0);

    //////////////   Area where pose traces will be added   ////////////////
    GtkWidget* traceLabel = gtk_label_new("Available Pose Traces:");
    gtk_box_pack_start(GTK_BOX(optionsBox_), traceLabel, FALSE, TRUE, 0);

    tracesBox_ = gtk_vbox_new(FALSE, 10);
    gtk_box_pack_start(GTK_BOX(optionsBox_), tracesBox_, FALSE, TRUE, 0);

    clearTracesButton_ = gtk_button_new_with_label("Clear Traces");
    gtk_box_pack_start(GTK_BOX(optionsBox_), clearTracesButton_, FALSE, TRUE, 0);

    g_signal_connect(clearTracesButton_,
                     "clicked",
                     G_CALLBACK(clear_traces_pressed),
                     static_cast<gpointer>(this));

    gridStatusBar_ = gtk_statusbar_new();
    gtk_box_pack_start(GTK_BOX(mainBox), gridStatusBar_, FALSE, TRUE, 0);

    gtk_widget_show(vxCanvas);    // XXX Show all causes errors!
    gtk_widget_show_all(window);
}

void clear_traces_pressed(GtkWidget* button, gpointer gui)
{
    BotGui* botGui = static_cast<BotGui*>(gui);
    botGui->clearAllTraces();
}
