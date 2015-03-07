#include <vector>
#include <exception>
#include <iostream>
using namespace std;

#include <DepthSense.hxx>
using namespace DepthSense;

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>

const long c_PIXEL_COUNT_QVGA = 76800; // 320x240
const long c_PIXEL_COUNT_VGA = 307200; // 640x480
const int c_MIN_Z = 100; // discard points closer than this
const int c_MAX_Z = 20000; // discard points farther than this

// Note that true range of camera is 0 - 31999

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;

uint32_t g_dFrames = 0; // depth frame number
uint32_t g_cFrames = 0; // color frame number
bool g_bDeviceFound = false;

uint8_t pixelsColor[3*c_PIXEL_COUNT_VGA]; // color map from color node

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);    // point cloud to save depth info 
pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");      // point cloud viewer
//pcl::visualization::ImageViewer c_viewer("Simple Color Viewer");      // image viewer

// event handler: new color frame
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
    cout << "color frame " << g_cFrames << "\n";
    for (int i = 0; i < c_PIXEL_COUNT_VGA; i++)
    {
        // Save color map
        pixelsColor[3*i] = data.colorMap[3*i+2];
        pixelsColor[3*i+1] = data.colorMap[3*i+1];
        pixelsColor[3*i+2] = data.colorMap[3*i];
    }

    //c_viewer.showRGBImage(pixelsColor,640,480); // display color map

    g_cFrames++;

}

// event handler: new depth frame
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
    cout << "depth frame " << g_dFrames << "\n";
    for (int i = 0; i < c_PIXEL_COUNT_QVGA; i++)
    {
        // use UV map to get indices of vertices corresponding to color map
        int xInd = int(data.uvMap[i].u*640);
        int yInd = int(data.uvMap[i].v*480);
        int linearInd = int(xInd+640*yInd);

        // display out of range points as black
    	if (data.vertices[i].z > c_MAX_Z || data.vertices[i].z < c_MIN_Z)
    	{
    	    cloud->points[i] = pcl::PointXYZRGB(0,0,0);
    	    continue;
    	}

        // map color to points that are within the specified range
        else if(linearInd>=0){
            uint8_t r = pixelsColor[3*linearInd];
            uint8_t g = pixelsColor[3*linearInd+1];
            uint8_t b = pixelsColor[3*linearInd+2];
            cloud->points[i] = pcl::PointXYZRGB(r,g,b);
        }

        //save input vertices to point cloud
        cloud->points[i].x = data.vertices[i].x;
        cloud->points[i].y = data.vertices[i].y;
        cloud->points[i].z = data.vertices[i].z;
    }

    // display point cloud
    viewer.showCloud(cloud);

    g_dFrames++;
}

// configure color node and output color map
void configureColorNode(Node node)
{
    if (node.is<ColorNode>() && !g_cnode.isSet())
    {
        // connect new color sample handler
        g_cnode = node.as<ColorNode>();
        g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);
    
        ColorNode::Configuration config = g_cnode.getConfiguration();
        config.frameFormat = FRAME_FORMAT_VGA;
        config.compression = COMPRESSION_TYPE_MJPEG;
        config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
        config.framerate = 25;
    
        g_cnode.setEnableColorMap(true);
    
        try
        {
            g_context.requestControl(g_cnode, 0);
            g_cnode.setConfiguration(config);
            cout << "color node connected\n";
        }
        catch (Exception& e)
        {
            cout << "ColorException: " << e.what() << "\n";
        }

        g_context.registerNode(node);
    }   
    
    
}


// configure depth map and output vertices and UV map
void configureDepthNode(Node node)
{
    if (node.is<DepthNode>() && !g_dnode.isSet())
    {
        g_dnode = node.as<DepthNode>();
        g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);
        DepthNode::Configuration config = g_dnode.getConfiguration();
        config.frameFormat = FRAME_FORMAT_QVGA;
        config.framerate = 25;
        config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
        config.saturation = true;
        g_dnode.setEnableVertices(true);
        g_dnode.setEnableUvMap(true);
        try
        {
            g_context.requestControl(g_dnode, 0);
            g_dnode.setConfiguration(config);
            cout << "depth node connected\n";
        }
        catch (Exception& e)
        {
            cout << "Depth Exception: " << e.what() << "\n";
        }
        g_context.registerNode(node);
    }
}

// event handler: setup feed
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureDepthNode(data.node);
    configureColorNode(data.node);
}

// event handler: tear down feed
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
    {
        g_dnode.unset();
        cout << "depth node disconnected\n";
    }
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
    {
        g_cnode.unset();
        cout << "color node disconnected\n";
    }
}

// event handler: device plugged in
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

// event handler: device unplugged
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    cout << "device disconnected\n";
}

int main(int argc, char** argv)
{
    g_context = Context::create("localhost");
    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);
    cloud->points.resize(c_PIXEL_COUNT_QVGA);

    // get list of devices already connected
    vector<Device> da = g_context.getDevices();

    // only use first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;
        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);
        vector<Node> na = da[0].getNodes();
        cout << "found " << (int)na.size() << " nodes\n";
        for (int n = 0; n < (int)na.size(); n++)
        {
            configureColorNode(na[n]);
            configureDepthNode(na[n]);
        }
    }

    g_context.startNodes();
    g_context.run();

    g_context.stopNodes();
    if (g_dnode.isSet())
    {
    	g_context.unregisterNode(g_dnode);
    }
    if (g_cnode.isSet())
    {
        g_context.unregisterNode(g_cnode);
    }
    return 0;
}

