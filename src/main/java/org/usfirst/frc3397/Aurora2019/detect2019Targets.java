/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3397.Aurora2019;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import com.kylecorry.frc.vision.Range;
import com.kylecorry.frc.vision.camera.CameraSettings;
import com.kylecorry.frc.vision.camera.FOV;
import com.kylecorry.frc.vision.camera.Resolution;
import com.kylecorry.frc.vision.contourFilters.ContourFilter;
import com.kylecorry.frc.vision.contourFilters.StandardContourFilter;
import com.kylecorry.frc.vision.filters.BrightnessFilter;
import com.kylecorry.frc.vision.filters.TargetFilter;
import com.kylecorry.frc.vision.targetConverters.TargetGrouping;
import com.kylecorry.frc.vision.targetConverters.TargetUtils;
import com.kylecorry.frc.vision.targeting.Target;
import com.kylecorry.frc.vision.targeting.TargetFinder;

import org.opencv.core.Mat;


/**
 * Add your docs here.
 */
public class detect2019Targets {
    /**
 * Detects the 2019 vision targets.
 * @param image The image from the camera.
 * @return The list of vision target groups.
 */
    public List<Target> detect2019Targets(Mat image){
    // Adjust these parameters for your team's needs
  
    // Target filter parameters
    double minBrightness = 200;
    double maxBrightness = 255;
  
    // Contour filter parameters
    Range area = new Range(0.03, 100);
    Range fullness = new Range(0, 100);
    Range aspectRatio = new Range(0.2, 4);
  
    // Camera settings
    FOV fov = new FOV(50, 40); // This is the approx. Microsoft LifeCam FOV
    Resolution resolution = new Resolution(640, 480);
    boolean cameraInverted = false;
  
    int imageArea = resolution.getArea();
  
    // An HSV filter may be better for FRC target detection
    TargetFilter filter = new BrightnessFilter(minBrightness, maxBrightness);
    ContourFilter contourFilter = new StandardContourFilter(area, fullness, aspectRatio, imageArea);
    CameraSettings cameraSettings = new CameraSettings(cameraInverted, fov, resolution);
    TargetFinder targetFinder = new TargetFinder(cameraSettings, filter, contourFilter, TargetGrouping.SINGLE);
  
    // Find the targets
    List<Target> targets = targetFinder.findTargets(image);
  
    // Sort the targets by x coordinates
    targets.sort(Comparator.comparingDouble(target -> target.getBoundary().center.x));
  
    List<Target> bays = new ArrayList<>();
    // If the current target is a left and the next is a right, make it a pair
    for (int i = 0; i < targets.size() - 1; i++) {
        Target current = targets.get(i);
        Target next = targets.get(i + 1);
  
        // Determine if the targets are a left and right pair
        if (isLeftTarget(current) && isRightTarget(next)){
            // Combine the targets
            Target bay = TargetUtils.combineTargets(current, next, cameraSettings);
            bays.add(bay);
            // Skip the next target
            i++;
        }
    }
  
    return bays;
  }
  
  /**
   * Determines if a target is a left vision target.
   * @param target The target.
   * @return True if it is a left target.
   */
  private boolean isLeftTarget(Target target){
      return target.getSkew() < 0;
  }
  
  /**
   * Determines if a target is a right vision target.
   * @param target The target.
   * @return True if it is a right target.
   */
  private boolean isRightTarget(Target target){
      return target.getSkew() > 0;
  }
}
