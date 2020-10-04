package org.firstinspires.ftc.teamcode.Navigation;

import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.NavigationTypesEnum;

public interface NavigationInterface
{

    public void activate();
    public void stop();
    public FieldPlacement getPlacement();
    public NavigationTypesEnum getType();

    //***//
    public FieldPlacement getTarget(String targetName);

    public boolean isActive();
}
