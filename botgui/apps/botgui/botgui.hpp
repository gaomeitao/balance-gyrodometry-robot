#ifndef APPS_BOTGUI_BOTGUI_HPP
#define APPS_BOTGUI_BOTGUI_HPP

#define ODOMETRY_POSE_CHANNEL "ODOMETRY_POSE"
#define BALANCEBOT_PATH_CHANNEL "CONTROLLER_PATH"

// util includes
#include <apps/utils/vx_gtk_window_base.hpp>
#include <common/pose_trace.hpp>
#include <map/occupancy_grid.hpp>

// lcm includes/types
#include <lcm/lcm-cpp.hpp>
#include <../lcmtypes/pose_xyt_t.hpp>
#include <../lcmtypes/robot_path_t.hpp>
#include <../lcmtypes/occupancy_grid_t.hpp>

// vx/gtk includes
#include <vx/vx_display.h>
#include <gtk/gtk.h>

// std lib includes
#include <atomic>
#include <map>
#include <mutex>
#include <string>


/**
* BotGui is the visual debugging tool for the Maebot.
*/
class BotGui : public VxGtkWindowBase
{
public:
    
    /**
    * Constructor for BotGui.
    *
    * \param    lcmInstance             Instance of LCM to use for subscription to and transmission of messages
    * \param    argc                    Count of command-line arguments for the program
    * \param    argv                    Command-line arguments for the program
    * \param    widthInPixels           Initial width of the GTK window to create
    * \param    heightInPixels          Initial height of the GTK window to create
    * \param    framesPerSecond         Number of frames per second to render visualizations
    *
    * \pre widthInPixels > 0
    * \pre heightInPixels > 0
    * \pre framesPerSecond > 0
    */
    BotGui(lcm::LCM* lcmInstance, int argc, char** argv, int widthInPixels, 
            int heightInPixels, int framesPerSecond);
    
    // GTK widget event handlers -- note these methods should only be called from the GTK event loop!
    void clearAllTraces(void);
    
    // VxGtkWindowBase interface -- GUI event handling
    int onMouseEvent(vx_layer_t* layer, 
                     vx_camera_pos_t* cameraPosition, 
                     vx_mouse_event_t* event, 
                     Point<float> worldPoint);// override;
    void onDisplayStart(vx_display_t* display);// override;
    void render(void);// override;

    // LCM Handlers
    void handleOccupancyGrid(const lcm::ReceiveBuffer* rbuf,
                             const std::string& channel,
                             const occupancy_grid_t* map);
    void handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                    const pose_xyt_t* pose);
    void handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                        const pose_xyt_t* odom);
    void handlePath(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                    const robot_path_t* path);

private:
    
    struct Trace
    {
        PoseTrace trace;
        GtkWidget* checkbox;
        const float* color;
    };
    
    std::map<std::string, Trace> traces_;           // Storage of all received pose traces

    OccupancyGrid map_;
    robot_path_t path_;                             // Current path being followed by the robot
    pose_xyt_t odometry_;                           // Most recent odometry measurement
    pose_xyt_t truepose_;

    bool havePath_;
    bool haveTruePose_;
    pose_xyt_t initialTruePose_;
    
    // Widgets w/variable input/output
    GtkWidget* showMapCheck_;                       // Checkbox indicating if the map should be drawn
    GtkWidget* showPathCheck_;                      // Checkbox indicating if the current path should be drawn
    GtkWidget* optionsBox_;                         // VBox holding all of the options widgets
    GtkWidget* tracesBox_;                          // VBox to append new PoseTrace checkboxes to
    GtkWidget* clearTracesButton_;                  // Button pressed when the user wants to clear any stored traces
    GtkWidget* gridStatusBar_;                      // Status bar to display some grid information
                                                    // enumeration
    std::atomic<bool> shouldResetStateLabels_;      // Flag indicating if the status of the state labels should be reset to the initial colors    
    std::atomic<bool> shouldClearTraces_;           // Flag indicating if all traces should be cleared on the next update
    std::vector<GtkWidget*> traceBoxesToAdd_;       // PoseTrace checkboxes that need to be added in the next render update    
    std::vector<const float*> traceColors_;         // Storage for the colors to assign to the various traces
    int nextColorIndex_;                            // Next color to assign to things
    
    Point<float> mouseWorldCoord_;                  // Global/world coordinate of the current mouse position
    Point<int>   mouseGridCoord_;                   // Grid cell the mouse is currently in
    
    lcm::LCM* lcmInstance_;                         // Instance of LCM to use for communication
    std::mutex vxLock_;                             // Mutex for incoming data -- LCM runs on different thread than GTK+
    
    // Additional helpers
    void addPose(const pose_xyt_t& pose, const std::string& channel);
    void destroyTracesIfRequested(void);
    void populateNewTraceBoxes(void);
    void updateGridStatusBarText(void);
    
    // VxGtkWindowBase interface -- GUI construction
    void createGuiLayout(GtkWidget* window, GtkWidget* vxCanvas);// override;
    
};

#endif // APPS_BOTGUI_BOTGUI_HPP
