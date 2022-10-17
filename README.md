# MotorMotion

This is an easy-to-use wrapper library around CTRE's [Phoenix](https://api.ctr-electronics.com/phoenix/release/cpp/) library as well as REV's [REVLib](https://codedocs.revrobotics.com/cpp/namespacerev.html) library.
The goal in this library is to abstract out the main motor controller interaction so you are not constantlly calling low-level functions and doing cookie-cutter math to calculate things like distance travelled/velocity/acceleration/etc.

## Using the Library

**For right now, only C++ is supported, contact us or make a pull request if you want to work on a Java implementation**

In order to get started with this library, you'll want to add it as a [Vendor Library](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html). Please follow the linked guide on adding a third party library, the link you will want to use is `[TODO: INSERT LINK HERE]`. For each release, we also will (automatically (soon)) publish a build on the Release page on the Github page for you to use as an offline install.

From here, you'll have access to the `TalonFXMotion`, `SparkMaxMotion`, and `TalonSRXMotion` classes by including it in your code.

### Examples
We also have written and included some example code for each of the motor motion classes. You'll be able to view them in the `examples/cpp` folder. Each project should be able to built as it's own robot project and deployed onto your robot (make sure to set your IDs/etc!)

## Development
If you use WPILib VSCode to develop on this plugin, it will probably end up asking you if you want to upgrade the project. Click `No`, otherwise it will try and convert the plugin project into a robot project and break everything.
If you want to build it (to make sure your code compiles), you can do `./gradlew build` (if you're using WPILib VSCode, The `WPILib: Build Robot Code` command does the same thing). Once you're ready to use it locally, do `./gradlew publish`. This will plop all of the required build files onto `build/repos/`.