package application;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;


/**
 * Moves the LBR in a start position, creates an FRI-Session and executes a
 * PositionHold motion with FRI overlay. During this motion joint angles and
 * joint torques can be additionally commanded via FRI.
 */
public class FRIOverlay extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "192.170.10.1";
    }

    @Override
    public void run()
    {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        // for torque mode, there has to be a command value at least all 5ms
        friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setReceiveMultiplier(1);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        int modeChoice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose control mode", "Torque", "Position", "Wrench");

        ClientCommandMode mode = ClientCommandMode.TORQUE;
        if (modeChoice == 0) {
            getLogger().info("Torque control mode chosen");
            mode = ClientCommandMode.TORQUE;
        }
        else if (modeChoice == 1) {
            getLogger().info("Position control mode chosen");
            mode = ClientCommandMode.POSITION;
        }
        else if (modeChoice == 2) {
            getLogger().warn("Wrench control mode not supported yet. Using position control mode instead");
            mode = ClientCommandMode.POSITION;
        }
        else {
            getLogger().warn("Invalid choice: using position control mode");
            mode = ClientCommandMode.POSITION;
        }
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, mode);

        int choice = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose stiffness for actuators", "0", "20", "50", "150", "300", "500");

        double stiffness = 0.;
        if (choice == 0) {
            getLogger().info("Stiffness of '0' chosen");
            stiffness = 0.;
        }
        else if (choice == 1) {
            getLogger().info("Stiffness of '20' chosen");
            stiffness = 20.;
        }
        else if (choice == 2) {
            getLogger().info("Stiffness of '50' chosen");
            stiffness = 50.;
        }
        else if (choice == 3) {
            getLogger().info("Stiffness of '150' chosen");
            stiffness = 150.;
        }
        else if (choice == 4) {
            getLogger().info("Stiffness of '300' chosen");
            stiffness = 300.;
        }
        else if (choice == 5) {
            getLogger().info("Stiffness of '500' chosen");
            stiffness = 500.;
        }
        else {
            getLogger().warn("Invalid choice: setting stiffness to '20'");
            stiffness = 20.;
        }

        // start PositionHold with overlay
        JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);
        if (mode == ClientCommandMode.TORQUE)
            ctrMode.setDampingForAllJoints(0.);

        try {
            PositionHold posHold = new PositionHold(ctrMode, -1, TimeUnit.SECONDS);
            getLogger().info("Robot is ready for ROS control.");
            _lbr.move(posHold.addMotionOverlay(jointOverlay));
        }
        catch(final CommandInvalidException e) {
            getLogger().error("ROS has been disconnected.");
        }

        // done
        friSession.close();
    }

    /**
     * main.
     *
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final FRIOverlay app = new FRIOverlay();
        app.runApplication();
    }

}
