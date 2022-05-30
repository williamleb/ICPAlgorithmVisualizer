#pragma once

/**
 * @file IcpGLCanvas.h
 *
 * @brief Une classe Canvas pour l'affichage de scènes avec OpenGL
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <nanogui/window.h>
#include <nanogui/glcanvas.h>

#include "IcpApplication.h"

/**
 * Une classe canvas OpenGL pour l'affichage de nuages de points
 */
class IcpGLCanvas : public nanogui::GLCanvas
{
public:
    IcpGLCanvas(IcpApplication* app);

    ~IcpGLCanvas();

    virtual void drawGL() override;

private:

    nanogui::GLShader m_shader;
    IcpApplication* m_app;

};
