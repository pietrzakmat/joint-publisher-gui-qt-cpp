#include <csignal>

#include "mainwindow.h"

QCoreApplication *appPtr;
void signalHandler(int signal)
{
    std::cout << "signalHandler(" << signal << ")" << std::endl;
    if (appPtr)
        appPtr->exit(128 + signal);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "myRobot_move_joint", ros::init_options::NoSigintHandler);

    QApplication app(argc, argv);
    appPtr = &app;
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    MainWindow window;
    window.show();

    // Event loop
    return app.exec();
}
