#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

#include <ignition/math/Vector2.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include <mutex>

namespace gazebo
{
    class SystemGUI : public SystemPlugin
    {
        /////////////////////////////////////////////
        /// \brief Destructor
    public: virtual ~SystemGUI()
        {
            this->connections.clear();
            if (this->userCam)
                this->userCam->EnableSaveFrame(false);
            this->userCam.reset();
            gui::MouseEventHandler::Instance()->RemovePressFilter("glwidget");
        }

        /////////////////////////////////////////////
        /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
        {
            this->connections.push_back(
                    event::Events::ConnectPreRender(
                            std::bind(&SystemGUI::Update, this)));

            gui::MouseEventHandler::Instance()->AddMoveFilter("glwidget",
                                                              std::bind(&SystemGUI::OnMousePress, this, std::placeholders::_1));
        }

        /////////////////////////////////////////////
        // \brief Called once after Load
    private: void Init()
        {
        }

        std::mutex mutexMouseClicked;
        ignition::math::Vector2i mouseClicked;
        bool isMouseClicked = false;
    private: bool OnMousePress(const common::MouseEvent& _event)
        {
            std::lock_guard<std::mutex> guard(mutexMouseClicked);
            if (_event.Button() == common::MouseEvent::LEFT){
                if (mouseClicked != _event.PressPos()) {
                    mouseClicked = _event.PressPos();
                    isMouseClicked = true;
                    std::cout << "Clicked at " << mouseClicked << " of screen " << std::endl;
                }
            }

            return true;

        }


        /////////////////////////////////////////////
        /// \brief Called every PreRender event. See the Load function.
    private: void Update()
        {

            if (!this->userCam)
            {
                // Get a pointer to the active user camera
                this->userCam = gui::get_active_camera();
            }

            // Get scene pointer
            rendering::ScenePtr scene = rendering::get_scene();

            // Wait until the scene is initialized.
            if (!scene || !scene->Initialized())
                return;

            if(isMouseClicked && this->userCam){
                std::lock_guard<std::mutex> guard(mutexMouseClicked);
                ignition::math::Vector3d position_clicked;
                scene->FirstContact(this->userCam, mouseClicked, position_clicked);
                std::cout<<"Clicked at "<<position_clicked<<" of world\n"<<std::endl;
                isMouseClicked = false;
            }

        }

        /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

        /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}