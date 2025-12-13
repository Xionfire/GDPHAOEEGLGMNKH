#pragma once
#include <utility>
#include <iostream>

/**
 * @brief CRTP base class for devices.
 * 
 * This class provides the common interface for all devices
 * using the Curiously Recurring Template Pattern (CRTP). 
 * More info of CRTP programing there https://stackoverflow.com/questions/4173254/what-is-the-curiously-recurring-template-pattern-crtp
 * It defines the public API: configure, alarmManagement, and cyclic.
 * 
 * Derived classes must implement the following protected methods:
 * - configureImpl(...)
 * - alarmManagementImpl(...)
 * - cyclicImpl(...)
 *  
 * @note In CRTP, these "Impl" methods must be defined in the header
 * (.h) file because the compiler needs their definitions during
 * template instantiation. If they are in a .cpp file, linker errors
 * like "undefined reference" will occur.
 * 
 * @tparam Derived The derived device class.
 */
template <typename Derived>
class IDevice
{
protected:
    /**
     * @brief Protected constructor to prevent direct instantiation.
     */
    IDevice()
    {
        std::cout << "[IDevice] constructor\n";
    }

    /**
     * @brief Protected destructor.
     */
    ~IDevice()
    {
        std::cout << "[IDevice] destructor\n";
    }

    /**
     * @brief Helper to get a reference to the derived class.
     * 
     * Provides a safe way to call methods in the derived class
     * without repeating static_cast everywhere.
     * 
     * @return Reference to Derived
     */
    Derived& derived() { return static_cast<Derived&>(*this); }

    /**
     * @brief Const helper to get a reference to the derived class.
     * 
     * @return Const reference to Derived
     */
    const Derived& derived() const { return static_cast<const Derived&>(*this); }

public:
    /**
     * @brief Configure the device.
     * 
     * Calls the derived class's configureImpl(...) method.
     * Supports variadic arguments to allow flexible parameters.
     * 
     * @tparam Args Variadic argument types
     * @param args Arguments forwarded to Derived::configureImpl
     */
    template <typename... Args>
    void configure(Args&&... args)
    {
        std::cout << "[Base] configure check (state == Init)\n";
        derived().configureImpl(std::forward<Args>(args)...);
    }

    /**
     * @brief Manage alarms for the device.
     * 
     * Calls the derived class's alarmManagementImpl(...) method.
     * Supports variadic arguments.
     * 
     * @tparam Args Variadic argument types
     * @param args Arguments forwarded to Derived::alarmManagementImpl
     */
    template <typename... Args>
    void alarmManagement(Args&&... args)
    {
        std::cout << "[Base] alarmManagement\n";
        derived().alarmManagementImpl(std::forward<Args>(args)...);
    }

    /**
     * @brief Cyclic operation called periodically.
     * 
     * Calls the derived class's cyclicImpl(...) method.
     * Supports variadic arguments.
     * 
     * @tparam Args Variadic argument types
     * @param args Arguments forwarded to Derived::cyclicImpl
     */
    template <typename... Args>
    void cyclic(Args&&... args)
    {
        std::cout << "[Base] cyclic check (state == Run)\n";
        derived().cyclicImpl(std::forward<Args>(args)...);
    }
};
