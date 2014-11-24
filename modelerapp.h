// DO NOT mess with this file.  If you do, animator will not work with
// your model, and you'll have to write a new one.  If you really really
// really need to do something here (unlikely) then don't complain if and
// when animator doesn't work.  -- Eugene
#ifndef MODELERAPP_H
#define MODELERAPP_H

#pragma warning(disable : 4018)
#pragma warning(disable : 4267)
#pragma warning(disable : 4311)
#pragma warning(disable : 4312)
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)

#include <vector>

#include "collidable.h"
#include "modelerview.h"

struct ModelerControl
{
	ModelerControl();
	ModelerControl(const char* name, float minimum, float maximum, float stepsize, float value);
	ModelerControl(const ModelerControl &o);
	ModelerControl& operator=(const ModelerControl &o);
	void SetVals(const char* name, float minimum, float maximum, float stepsize, float value);

	char  m_name[128];
	float m_minimum;
	float m_maximum;
	float m_stepsize;
	float m_value;
};

// Forward declarations for ModelerApplication
class ModelerView;
class ModelerUI;
class Fl_Box;
class Fl_Slider;
class Fl_Value_Slider;
class ParticleSystem;

// The ModelerApplication is implemented as a "singleton" design pattern,
// the purpose of which is to only allow one instance of it.
class ModelerApplication
{
public:
	~ModelerApplication();

	// Fetch the global ModelerApplication instance
	static ModelerApplication* Instance();

    // Initialize the application; see sample models for usage
	void Init(ModelerViewCreator_f createView, 
              const ModelerControl controls[], 
              unsigned numControls); 

    // Starts the application, returns when application is closed
	int  Run();

    // Get a pointer to the UI
	ModelerUI* GetUI() { return m_ui; };

	// Get and set slider values.
    double GetControlValue(int controlNumber);
    void   SetControlValue(int controlNumber, double value);

	// Get and set particle system
	ParticleSystem *GetParticleSystem();
	void SetParticleSystem(ParticleSystem *s);

    // Manage collidable objects.
    const std::vector<Collidable*> GetCollidables() const;
    void AddCollidable(Collidable *collidable);
    void ResetCollidables();
    
    const std::vector<Collision> GetCollisions() const;
    void AddCollision(Collision collision);
    
	// Return the current time
	float GetTime();

	// Return the current fps
	int GetFps();

	// Returns animating flag
	bool Animating();

private:
	// Private for singleton
	ModelerApplication() : m_numControls(-1) { ps = 0; }
	ModelerApplication(const ModelerApplication&) {}
    ModelerApplication& operator=(const ModelerApplication&) { return *this; }
	
	// The instance
	static ModelerApplication *m_instance;

	ModelerUI *m_ui;
	int					  m_numControls;

    static void ValueChangedCallback();
	static void RedrawLoop(void*);

	// Just a flag for updates
	bool m_animating;

	// Particle System variables
	ParticleSystem *ps;
    
    // Collidable object variables
    std::vector<Collidable*> collidables;
    std::vector<Collision> collisions;
};

#endif
