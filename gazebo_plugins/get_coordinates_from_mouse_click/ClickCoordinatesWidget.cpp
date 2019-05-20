#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "ClickCoordinatesWidget.h"
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <gazebo/rendering/rendering.hh>

#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(ClickCoordinatesWidget)

/////////////////////////////////////////////////
ClickCoordinatesWidget::ClickCoordinatesWidget()
        : GUIPlugin()
{

    // Set the frame background and foreground colors
    this->setStyleSheet(
            "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    // Create the layout that sits inside the frame
    QVBoxLayout *frameLayout = new QVBoxLayout();


    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

    gui::MouseEventHandler::Instance()->AddMoveFilter("my_plugin",
                                                      std::bind(&ClickCoordinatesWidget::OnMousePress, this, std::placeholders::_1));


    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);


    this->mouseClicked = false;

}

/////////////////////////////////////////////////
ClickCoordinatesWidget::~ClickCoordinatesWidget()
{
}


bool ClickCoordinatesWidget::OnMousePress(const common::MouseEvent& _event)
{
    if (_event.Button() == common::MouseEvent::LEFT){
        if (this->mouseClicked != _event.PressPos()) {
            this->mouseClicked = _event.PressPos();
            this->isMouseClicked = true;
            std::cout << "Clicked at " << this->mouseClicked << " of screen " << std::endl;

            rendering::UserCameraPtr userCam = gui::get_active_camera();
            rendering::ScenePtr scene = rendering::get_scene();

            // Wait until the scene is initialized.
            if (!scene || !scene->Initialized())
                return true;

            if(this->isMouseClicked && userCam){
                ignition::math::Vector3d position_clicked;
                scene->FirstContact(userCam, mouseClicked, position_clicked);
                std::cout<<"Clicked at "<<position_clicked<<" of world\n"<<std::endl;
                this->isMouseClicked = false;
            }

        }
    }

    return false;
}
