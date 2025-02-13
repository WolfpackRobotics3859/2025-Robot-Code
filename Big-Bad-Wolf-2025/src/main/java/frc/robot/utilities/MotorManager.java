package frc.robot.utilities;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorManager 
{
    private static ArrayList<Pair<String, TalonFX>> m_Motors;

    public static boolean AddMotor(String name, int id)
    {
        if(m_Motors == null)
        {
            m_Motors = new ArrayList<Pair<String, TalonFX>>();
        }

        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getFirst() == name)
            {
                System.out.println("[ERROR] Attempted to add a motor with duplicate name.");
                for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
                    System.out.println(ste + "\n");
                }
                return false;
            }

            if(pair.getSecond().getDeviceID() == id)
            {
                System.out.println("[ERROR] Attempted to add a motor with duplicate id.");
                for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
                    System.out.println(ste + "\n");
                }
                return false;
            }
        }

        boolean returnValue = m_Motors.add(new Pair<String, TalonFX>(name, new TalonFX(id)));

        if(returnValue)
        {
            InitializeSmartdashboardFields(GetMotor(name));
        }

        return returnValue;
    }

    public static TalonFX GetMotor(String name)
    {
        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getFirst() == name)
            {
                return pair.getSecond();
            }
        }

        System.out.println("[ERROR] No motor with name " + name +  " found in " + MotorManager.class.getName());
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }

        return null;
    }

    public static TalonFX GetMotor(int id)
    {
        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getSecond().getDeviceID() == id)
            {
                return pair.getSecond();
            }
        }

        System.out.println("[ERROR] No motor with id " + id +  " found in " + MotorManager.class.getName());
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }

        return null;
    }

    public static int GetMotorID(String name)
    {
        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getFirst() == name)
            {
                return pair.getSecond().getDeviceID();
            }    
        }

        System.out.println("[ERROR] No motor with name " + name +  " found in " + MotorManager.class.getName());
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }

        return 404;
    }

    public static String GetMotorName(int id)
    {
        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getSecond().getDeviceID() == id)
            {
                return pair.getFirst();
            }
        }

        System.out.println("[ERROR] No motor with id " + id +  " found in " + MotorManager.class.getName());
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }

        return null;
    }

    public static String GetMotorName(TalonFX motor)
    {
        for (Pair<String,TalonFX> pair : m_Motors) 
        {
            if(pair.getSecond() == motor)
            {
                return pair.getFirst();
            }    
        }

        System.out.println("[ERROR] Failed to find the name of the given motor.");
        for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
            System.out.println(ste + "\n");
        }

        return null;
    }

    public static boolean ApplyConfigs(TalonFXConfiguration configs, String name)
    {
        TalonFX motor = GetMotor(name);

        if(motor == null)
        {
            System.out.println("[ERROR] Motor configuration failed for motor named " + name + " due to not finding motor.");
            return false;
        }

        return ApplyConfigToMotor(motor, configs);
    }

    public static boolean ApplyConfigs(TalonFXConfiguration configs, int id)
    {
        TalonFX motor = GetMotor(id);

        if(motor == null)
        {
            System.out.println("[ERROR] Motor configuration failed for motor with id " + id + " due to not finding motor.");
            return false;
        }

        return ApplyConfigToMotor(motor, configs);
    }

    public static boolean ApplyControlRequest(ControlRequest request, int id)
    {
        TalonFX motor = GetMotor(id);
        
        if(motor == null)
        {
            System.out.println("[ERROR] Failed to apply control request due to not being able to find motor.");
            return false;
        }

        motor.setControl(request);

        String prefix = SmartDashboardKeyPrefix(motor);

        SmartDashboard.putString(prefix + "Control Mode", request.getControlInfo().get("Name"));

        if(request.getControlInfo().containsKey("Position"))
        {
            SmartDashboard.putString(prefix + "Control Value", request.getControlInfo().get("Position"));
        }

        if(request.getControlInfo().containsKey("Output"))
        {
            SmartDashboard.putString(prefix + "Control Value", request.getControlInfo().get("Output"));
        }

        return true;
    }

    public static BooleanSupplier InPosition(int id, double tolerance)
    {
        TalonFX motor = GetMotor(id);

        if(motor == null)
        {
            System.out.println("[ERROR] Failed to get closed loop data due to not being able to find motor.");
            return () -> false;
        }

        return () -> Math.abs(motor.getClosedLoopError().getValueAsDouble()) < tolerance;
    }

    private static String SmartDashboardKeyPrefix(TalonFX motor)
    {
        return GetMotorName(motor) + " [" + motor.getDeviceID() + "] ";
    }

    private static void InitializeSmartdashboardFields(TalonFX motor)
    {
        String prefix = SmartDashboardKeyPrefix(motor);
        SmartDashboard.putNumber(prefix + "Motor Position", 0);
        SmartDashboard.putString(prefix + "Control Mode", "N/A");
        SmartDashboard.putString(prefix + "Control Value", "N/A");
    }

    private static boolean ApplyConfigToMotor(TalonFX motor, TalonFXConfiguration config)
    {
        if(!motor.getConfigurator().apply(config).isOK())
        {
            System.out.println("[ERROR] Motor configuration failed.");
            for (StackTraceElement ste : Thread.currentThread().getStackTrace()) {
                System.out.println(ste + "\n");
            }
            return false;
        }

        return true;
    }

}
