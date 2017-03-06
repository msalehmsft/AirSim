#pragma once

namespace MavLinkUwp
{
    public ref class UwpMavLink sealed
    {
        std::shared_ptr<mavlinkcom::MavLinkConnection> _com;
    public:
        UwpMavLink();
        bool connectToMavLink();

    };
}
