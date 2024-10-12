package org.firstinspires.ftc.teamcode.utils;

public class ConfigurationException extends Exception
{
    public ConfigurationException (String err)
    {
        super("Configuration Error: " + err);
    }
}
