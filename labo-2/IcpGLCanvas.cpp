/**
 * @file IcpGLCanvas.h
 *
 * @brief Une classe Canvas pour l'affichage de scènes avec OpenGL
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "IcpGLCanvas.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace nanogui;

IcpGLCanvas::IcpGLCanvas(IcpApplication* app) : nanogui::GLCanvas(app->getWindow()), m_app(app) {

    // Un m_shader simple pour l'affichage de nuages de points.
    m_shader.init(
        // Shader name
        "point_cloud_shader",

        // Vertex m_shader
        "#version 410\n"
        "uniform mat4 modelViewProj;\n"
        "uniform vec4 color;\n"
        "in vec3 position;\n"
        "void main() {\n"
        "    gl_Position = modelViewProj * vec4(position, 1.0);\n"
        "}",

        // Fragment m_shader
        "#version 410\n"
        "uniform vec4 color;\n"
        "out vec4 frag_color;\n"
        "void main() {\n"
        "    frag_color = color;\n"
        "}"
    );

    m_shader.bind();
    m_shader.uploadAttrib("position", m_app->getReferencePoints());
    m_shader.setUniform("color", Eigen::Vector4f(0.0f, 0.0, 1.0f, 1.0f));
}

IcpGLCanvas::~IcpGLCanvas() {
    m_shader.free();
}

void IcpGLCanvas::drawGL()
{
    m_shader.bind();

    // Matrice de la caméra
    const Matrix4f viewMat = lookAt(Vector3f(-2.0f, 4.0f, -10.0f), Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    // Matrice de vue, calculée d'après des paramètres de type gluPerspective
    //
    const float fov = 60.0f;
    const float n = 0.1f;
    const float f = 100.0f;
    const float t = n * std::tan(0.5f * fov * ((float)M_PI / 180.0f));
    const float b = -t;
    const float r = t * this->width() / this->height();
    const float l = -r;
    const Matrix4f projMat = frustum(l, r, b, t, n, f);

    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(4);

    // Affichage du premier nuage de points
    Matrix4f mvp = projMat * viewMat;
    m_shader.setUniform("modelViewProj", mvp);
    m_shader.setUniform("color", Eigen::Vector4f(1.0f, 0.0, 0.0f, 1.0f));
    m_shader.uploadAttrib("position", m_app->getReferencePoints());
    m_shader.drawArray(GL_POINTS, 0, m_app->getReferencePoints().cols());

    const gti320::Matrix4d M = m_app->getSrcTransform() * m_app->getInitTransform();

    // HACK ! NanoGUI requiert des matrices dont le type est défini par la
    // librairie Eigen. On doit donc définir une nouvelle matrice (eigen) et y
    // copier le contenu de notre matrice (gti320::Matrix4d)
    Matrix4f model;
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 4; ++i)
            model(i, j) = M(i, j);

    // Affichage du second nuage de points
    mvp = projMat * viewMat * model;
    m_shader.setUniform("modelViewProj", mvp);
    m_shader.setUniform("color", Eigen::Vector4f(0.0f, 0.0, 1.0f, 1.0f));
    m_shader.uploadAttrib("position", m_app->getSourcePoints());
    m_shader.drawArray(GL_POINTS, 0, m_app->getSourcePoints().cols());

    glDisable(GL_DEPTH_TEST);
}
