//
// Created by fanziqi on 2022/11/12.
//

#ifndef CHEETAH_SOFTWARE_CYBERDOGPARAMS_H
#define CHEETAH_SOFTWARE_CYBERDOGPARAMS_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Cyberdog
 */
template<typename T>
Quadruped<T> buildCyberdog()
{
    Quadruped<T> cheetah;
    cheetah._robotType = RobotType::MINI_CHEETAH;
    
    // Parameter
    cheetah._bodyMass = 6.52;
    cheetah._bodyLength = 0.47072;
    cheetah._bodyWidth = 0.1;
    cheetah._bodyHeight = 0.1;
    cheetah._abadGearRatio = 6;
    cheetah._hipGearRatio = 6;
    cheetah._kneeGearRatio = 9.0;
    cheetah._abadLinkLength = 0.10715;
    cheetah._hipLinkLength = 0.2;
    cheetah._kneeLinkLength = 0.217;
    cheetah._kneeLinkY_offset = 0.0;
    cheetah._maxLegLength = 0.409;
    
    cheetah._motorTauMax = 3.5;
    cheetah._batteryV = 24.0;
    cheetah._motorKT = 0.05;
    cheetah._motorR = 0.173;
    cheetah._jointDamping = 0.01;
    cheetah._jointDryFriction = 0.2;
    
    /* Spatial Inertia */
    MassProperties<T> abadMassProperties;
    abadMassProperties << 5.090000033378601e-01, -2.290499862283468e-03, 9.161999914795160e-04, -2.545000053942204e-03, 3.809741465374827e-04, 6.938322330825031e-04, 4.733563982881606e-04, 5.070999577583279e-06, -1.165249886980746e-05, 1.252289894182468e-05;
    cheetah._abadInertia = SpatialInertia<T>(abadMassProperties);
    MassProperties<T> hipMassProperties;
    hipMassProperties << 6.639999747276306e-01, -1.925599877722561e-03, -2.217759937047958e-02, -1.235039997845888e-02, 3.337649162858725e-03, 2.638501580804586e-03, 1.309316023252904e-03, -9.303330443799496e-06, -1.928161655087024e-04, -7.150374585762620e-07;
    cheetah._hipInertia = SpatialInertia<T>(hipMassProperties);
    MassProperties<T> kneeMassProperties;
    kneeMassProperties << 1.140000000596046e-01, 8.093999931588769e-04, 4.559999979392160e-06, -1.281473971903324e-02, 1.455305144190788e-03, 2.152151660993695e-03, 7.054469315335155e-04, 5.125896223034943e-07, 8.388462447328493e-05, -3.237600054717404e-08;
    cheetah._kneeInertia = SpatialInertia<T>(kneeMassProperties);
    MassProperties<T> abadRotorMassProperties;
    abadRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 6.299999949987978e-05, 3.300000025774352e-05, 3.300000025774352e-05, 0.000000000000000e+00, 1.311341551839262e-12, 0.000000000000000e+00;
    cheetah._abadRotorInertia = SpatialInertia<T>(abadRotorMassProperties);
    MassProperties<T> hipRotorMassProperties;
    hipRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 3.300000025774352e-05, 6.299999949987978e-05, 3.300000025774352e-05, -1.311341551839262e-12, 0.000000000000000e+00, 0.000000000000000e+00;
    cheetah._hipRotorInertia = SpatialInertia<T>(hipRotorMassProperties);
    MassProperties<T> kneeRotorMassProperties;
    kneeRotorMassProperties << 5.499999970197678e-02, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 3.300000025774352e-05, 6.299999949987978e-05, 3.300000025774352e-05, -1.311341551839262e-12, 0.000000000000000e+00, 0.000000000000000e+00;
    cheetah._kneeRotorInertia = SpatialInertia<T>(kneeRotorMassProperties);
    MassProperties<T> bodyMassProperties;
    bodyMassProperties << 6.519999980926514e+00, 1.284440010786057e-01, -1.564799924381077e-03, -5.737599916756153e-03, 3.205142542719841e-02, 1.370674073696136e-01, 1.494587212800980e-01, 5.662297917297110e-05, 2.728030551224947e-03, -2.321734355064109e-04;
    cheetah._bodyInertia = SpatialInertia<T>(bodyMassProperties);
 
  //Vec3<T> rotorCOM(0, 0, 0);


    // locations
    cheetah._abadRotorLocation = Vec3<T>(2.353599965572357e-01, 5.000000074505806e-02, 0);//test without this shit / normal value 
    cheetah._abadLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0)* 0.5;
    cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
    cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
    cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
    cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);
    
    return cheetah;
}

#endif // CHEETAH_SOFTWARE_CYBERDOGPARAMS_H
