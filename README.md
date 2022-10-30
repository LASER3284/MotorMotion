# MotorMotion

This is an easy-to-use wrapper library around CTRE's [Phoenix](https://api.ctr-electronics.com/phoenix/release/cpp/) library as well as REV's [REVLib](https://codedocs.revrobotics.com/cpp/namespacerev.html) library.
The goal in this library is to abstract out the main motor controller interaction so you are not constantlly calling low-level functions and doing cookie-cutter math to calculate things like distance travelled/velocity/acceleration/etc.

## Using the Library

**For right now, only C++ is supported, contact us or make a pull request if you want to work on a Java implementation**

In order to get started with this library, you'll want to add it as a [Vendor Library](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html). **It is also important to install REVLib, CTRE Phoenix, and WPILib New Commands, otherwise the library won't be able to access the APIs for the actual motors.** Please follow the linked guide on adding a third party library, the link you will want to use is `[TODO: INSERT LINK HERE]`. For each release, we also will (automatically (soon)) publish a build on the Release page on the Github page for you to use as an offline install.

From here, you'll have access to the `TalonFXMotion`, `SparkMaxMotion`, and `TalonSRXMotion` classes by including it in your code.

### Usage

Each class can be instantiated using CAN device IDs for the motor just as any other class. **Do NOT instantiate the `laser::MotorMotion<ErrorEnum, MotorType>` class, it will not work because it does not have a defined constructor and you won't have a motor pointer available.** This is due to the fact that it is meant to be a template class that defines virtual methods that get overridden by the derived classes. The reason for this is so that other motor controllers are more easily supported by the library. For example, `TalonFXMotion` is derived from `MotorMotion<ctre::phoenix::ErrorCode, ctre::phoenix::*::WPI_TalonFX>` (the star shows that some of the namespace was omitted).

Every class is within the `laser` namespace, and each motor controller type has its own namespace within `laser` (e.g, for TalonFX, the namespace would be `laser::talonfx`, which will then contain the `laser::talonfx::TalonFXMotion` class). The enums used for the state and setpoint type are also in the base `laser` namespace. 

This library makes use of the Units library provided through WPILib, and it is important that this is used so that method overloads for position, linear velocity, and angular velocity can be made. It is recommended to also use the Units library for unit-based math outside of this library, as it will make interfacing with this library easier.

### Examples

We also have written and included some example code for each of the motor motion classes. You'll be able to view them in the `examples/cpp` folder. Each project should be able to built as it's own robot project and deployed onto your robot (make sure to set your IDs/etc!)

## Documentation

There is full API documentation both on our website `[TODO: insert link]` and under the `../MotorMotion-doc` directory. For local documentation, clone [jothepro/doxygen-awesome-css](https://github.com/jothepro/doxygen-awesome-css) in this repo's root; from there you can run Doxygen from the command line or open Doxywizard an open the local `Doxyfile`. To open local docs, go to `../MotorMotion-doc` and open `index.html`. You can also clone our `docs` branch.

## Development

If you use WPILib VSCode to develop on this plugin, it will probably end up asking you if you want to upgrade the project. Click `No`, otherwise it will try and convert the plugin project into a robot project and break everything.
If you want to build it (to make sure your code compiles), you can do `./gradlew build` (if you're using WPILib VSCode, The `WPILib: Build Robot Code` command does the same thing). Once you're ready to use it locally, do `./gradlew publish`. This will plop all of the required build files onto `build/repos/`.

### License

MotorMotion uses the GNU Lesser General Public License (GNU LGPL). This means that any derived libraries or projects using direct source code from this library must follow the GNU LGPL; however, this does not apply to users of the library, such as those using the headers for their robot code. 

Copyright 2022 Camdenton LASER 3284

### Contributors

Charlotte Patton (2022-present),
Thomas Iliff (2022-present)

### Current Features
* Support for Falcon 500 motors, using the TalonFX motor controller
* Support for NEO and NEO 550 motors, using the CAN SparkMax motor controller
* Three modes of setpoint:
  * Positional (distance traveled)
  * Linear velocity (wheel speed)
  * Angular velocity (rotational speed)
* Usage of Units library through WPILib
* Configurable motor behavior
  * Limit switch support (through the motor controller)
  * Linear velocity and position based on wheel size and gear ratio

### Planned Features
* Support for TalonSRX brushed DC motor controller
* Support for external limit switches
* Support for external encoders
* Support for simulation
* Soft limits

### TODO
* Fix build errors
  * Fix laser/SparkMaxMotion.h
  * Fix SparkMaxMotion.cpp
* Document code
  * Comments
  * Documentation files
* Test on hardware