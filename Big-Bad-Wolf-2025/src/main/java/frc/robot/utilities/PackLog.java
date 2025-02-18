package frc.robot.utilities;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.utilities.packLog.LogType;

/**
 * A helper class used to provide standard formatting for logs and to cache component names for easy and identifiable logs.
 * 
 * Ensure that {@link DataLogManager} has been started before logging anything.
 */
public class PackLog 
{
    private String m_ModuleName;

    /**
     * Creates a new PackLog instance with a cached name to tag all new log messages with.
     * @param moduleName
     */
    public PackLog(String moduleName)
    {
        this.m_ModuleName = moduleName;
    }

    public void Log(String message)
    {
        Log(LogType.INFO, message);
    }

    public void Log(LogType type, String message)
    {
        LogEntry(type, this.m_ModuleName, message);
    }

    public void Log(LogType type, String message, StackTraceElement[] trace)
    {
        this.Log(type, message);
        LogTrace(trace);
    }

    public static void LogTrace(StackTraceElement[] trace)
    {
        for (StackTraceElement ste : trace) 
        {
            DataLogManager.log(ste.toString());
        }
    }

    public static void LogTrace(String message, StackTraceElement[] trace)
    {
        DataLogManager.log(message);
        LogTrace(trace);
    }

    private static void LogEntry(LogType type, String module, String message)
    {
        StringBuilder log = new StringBuilder("[" + type.name() + "]");
        log.append(" " + module + ": ");
        log.append(message);

        DataLogManager.log(log.toString());
    }
}
