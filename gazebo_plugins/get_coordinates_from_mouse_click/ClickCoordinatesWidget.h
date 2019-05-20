#ifndef _GUI_CLICK_COORDINATES_WIDGET_HH_
#define _GUI_CLICK_COORDINATES_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
    class GAZEBO_VISIBLE ClickCoordinatesWidget : public GUIPlugin
    {
        Q_OBJECT

        /// \brief Constructor
        /// \param[in] _parent Parent widget
    public: ClickCoordinatesWidget();

        /// \brief Destructor
    public: virtual ~ClickCoordinatesWidget();

    private: bool OnMousePress(const common::MouseEvent& _event);

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    private: ignition::math::Vector2i mouseClicked;
    private: bool isMouseClicked;
    };
}
#endif