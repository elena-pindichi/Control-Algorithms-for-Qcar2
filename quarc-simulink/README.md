# Connection with the car
## Model configuration settings:
1. Once you open Simulink, open the Model Configuration Settings.
2. Under Solver->Solver selection, set the Type Fixed Step and set the solver to the desired one. 
3. Under Solver->Solver details set the Fixed-step size to the desired value.
4. Under Code Generation->Target selection set System target file as applicable to your deployed application:
    - For applications running on the development machine, use quarc_win64.tlc for a windows-based platform. This may be used for real-time simulations and robust communications.
    - For applications to be deployed to a QCar target, select the quarc_linux_nvidia.tlc for the QCar platform.
5. If the target is the QCar, you must also specify the location of the platform on the network so that the application can be downloaded there automatically by Quarc. Navigate to Code Generation->Interface and edit the MEX file arguments by adding the following code:
    ```'-w -d /tmp -uri  %u','tcpip://IP_ADDRESS:PORT'```
Do not exclude the comma or the single quotations. Here IP_ADDRESS refers to the IPv4 address of the QCar shown on the car's LCD and screen PORT refers to the number between 17001 and 17999. Simulink will use the PORT to communicate with the deployed application and display any data connected to Simulink sinks in your code on your local machine itself.

For the project I have worked on, it worked:
    ```'-w -d /tmp -uri %u', 'tcpip://192.168.1.126:17001'```

## Code generation, deployment and monitoring:
1. Build and download your code. Press Ctrl+B, open Diagnostic viewer.
2. Connect to the target Ctr+Shift+O.
3. Start your model: Ctrl+Shift+Q.
4. Stop your model: Ctrl+Shift+W.

## Blocks in Simulink
More details regarding the blocks to add in Simulink can be found here:
https://www.quanser.com/resource/quarc-essentials-hardware-interfacing/



Additional Information: https://github.com/quanser/ACC-Competition-2025.git
