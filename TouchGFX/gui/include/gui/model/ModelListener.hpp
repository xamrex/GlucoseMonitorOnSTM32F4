#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}
    virtual void DrawPoint () {};
    virtual void UpdateCircle() {};
    virtual void ShowStep () {};
    virtual void UpdateDelta () {};
    virtual void FadeBGin() {};
    virtual void FadeBGout() {};
    void bind(Model* m)
    {
        model = m;
    }
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
