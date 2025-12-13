#pragma once
#include "ISharedDevice.h"

/**
 * @brief Sensor device class implementing IDevice CRTP interface.
 * 
 * This class inherits from IDevice<Sensor> using CRTP. It provides
 * the concrete implementations of the required "Impl" methods:
 * configureImpl, alarmManagementImpl, and cyclicImpl.
 * 
 * @note In CRTP, these "Impl" methods must be visible to the compiler
 * at the point where IDevice<Sensor> is instantiated. That is why
 * they are defined in the header (.h) file instead of a .cpp file.
 * Moving them to a .cpp file would cause linker errors like
 * "undefined reference" because the compiler needs the definitions
 * during template instantiation.
 */
class Sensor : public IDevice<Sensor>
{
public:
    /// Allow IDevice base class to access protected Impl methods
    friend class IDevice<Sensor>;

    /// Constructor
    Sensor() { std::cout << "[Sensor] constructor\n"; }

    /// Destructor
    ~Sensor() { std::cout << "[Sensor] destructor\n"; }

protected:
    /**
     * @brief Concrete implementation of configure.
     * 
     * @param dummy
     */
    void configureImpl(std::string dummy) {
        std::cout << "[Sensor] configureImpl(" << dummy << ")\n";
    }
    
    
    /**
     * @brief Concrete implementation of alarm management.
     * 
     * @param dummy 
     */
    void alarmManagementImpl(std::string dummy) {
        std::cout << "[Sensor] alarmManagementImpl(" << dummy << ")\n";
    }


    /**
     * @brief Concrete implementation of cyclic behavior.
     * 
     * This method should called periodically by the cyclic process.
     */
    void cyclicImpl() {
        std::cout << "[Sensor] cyclicImpl()\n";
        alarmManagementImpl("dummy");
    }

};
