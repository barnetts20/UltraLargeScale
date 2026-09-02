// Fill out your copyright notice in the Description page of Project Settings.


#include "StarSystemDataGenerator.h"

#pragma region Star System Generator
FVector StarSystemDataGenerator::GetOrbitPosition(const FOrbit& Orbit) const
{
	double a = Orbit.SemiMajorAxis;
	double e = Orbit.Eccentricity;
	double b = a * FMath::Sqrt(1.0 - e * e);

	// Build orbital basis vectors
	FVector UpRef = FVector::UpVector;
	FVector OrbitRight = FVector::CrossProduct(Orbit.Normal, UpRef);
	if (OrbitRight.IsNearlyZero())
		OrbitRight = FVector::CrossProduct(Orbit.Normal, FVector::ForwardVector);
	OrbitRight.Normalize();

	FVector OrbitForward = FVector::CrossProduct(Orbit.Normal, OrbitRight).GetSafeNormal();

	// Calculate position on ellipse
	double theta = Orbit.Phase;
	FVector Pos = Orbit.Center
		+ OrbitRight * (a * FMath::Cos(theta))
		+ OrbitForward * (b * FMath::Sin(theta));

	return Pos;
}
#pragma endregion