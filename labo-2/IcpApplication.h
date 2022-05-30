#pragma once

/**
 * @file IcpApplication.h
 *
 * @brief Partie applicative du Laboratoire #2 : 
 *         - Implémentation de l'algorithme ICP
 *         - Gestion de l'interface graphique
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include <nanogui/screen.h>

#include "Icp.h"

class IcpGLCanvas;

/**
 * Classe IcpGLApplication
 *
 * Implémentation de l'algorithme ICP (iterative closest point)
 * et gestion des fenêtres correspondants du GUI.
 */
class IcpApplication : public nanogui::Screen
{
public:
	IcpApplication(const char* objFolder = "");

	virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) override;

	/**
	 * Réaction lorsqu'un des slider est modifier
	 */
	void onSlidersChanged();

	/**
	 * Getters
	 */
	nanogui::Window* getWindow() const { return m_window; }

	const gti320::Matrix4d getSrcTransform() const { return m_srcTM; }

	const gti320::Matrix4d getInitTransform() const { return m_initTM; }

	const gti320::Points3d& getReferencePoints() const { return m_refPoints; }

	const gti320::Points3d& getSourcePoints() const { return m_sourcePoints; }

private:
	/**
	 * Charge un nuage de points à partir d'un fichier Wavefront OBJ.
	 *
	 * @param filename  nom du fichier à charger
	 */
	void loadModel(const std::string& filename);

	/**
	 * Effectue une itération de l'algorithme ICP
	 */
	void stepIcp();

	/**
	 * Ramène le nuage de points source dans sa configuration initiale.
	 *
	 * Les transformations calculées par l'ICP et les modifications effectuées
	 * par l'utilisateur sont annulées.
	 */
	void reset();


	/**
	 * Calcule l'erreur d'alignement entre les deux nuages de points
	 */
	void computeError();

	IcpGLCanvas* m_canvas;
	nanogui::Window* m_window;

	// Panels
	Widget* m_panelTranslateX;
	Widget* m_panelTranslateY;
	Widget* m_panelTranslateZ;
	Widget* m_panelRotateX;
	Widget* m_panelRotateY;
	Widget* m_panelRotateZ;
	Widget* m_panelError;

	// Textboxes 
	nanogui::TextBox* m_textboxTranslateX;
	nanogui::TextBox* m_textboxTranslateY;
	nanogui::TextBox* m_textboxTranslateZ;
	nanogui::TextBox* m_textboxRotateX;
	nanogui::TextBox* m_textboxRotateY;
	nanogui::TextBox* m_textboxRotateZ;
	nanogui::TextBox* m_textboxError;

	// Labels
	nanogui::Label* m_labelTranslateX;
	nanogui::Label* m_labelTranslateY;
	nanogui::Label* m_labelTranslateZ;
	nanogui::Label* m_labelRotateX;
	nanogui::Label* m_labelRotateY;
	nanogui::Label* m_labelRotateZ;
	nanogui::Label* m_labelError;

	// Sliders
	nanogui::Slider* m_sliderTranslateX;
	nanogui::Slider* m_sliderTranslateY;
	nanogui::Slider* m_sliderTranslateZ;
	nanogui::Slider* m_sliderRotateX;
	nanogui::Slider* m_sliderRotateY;
	nanogui::Slider* m_sliderRotateZ;

	// Les nuages de points
	//
	// Nuage de référence, il s'agit des points tels que lus dans le fichier .obj
	gti320::Points3d m_refPoints;
	//
	// Nuage source, il s'agit des points transformés. Le but des l'ICP est de
	// ramener ces points sur ceux de référence.
	gti320::Points3d m_sourcePoints;

	gti320::Vector3d m_rotation; // rotation déterminée par l'utilisateur via le GUI
	gti320::Vector3d m_translation; // translation déterminée par l'utilisation via le GUI
	gti320::Matrix4d m_srcTM; // transformation rigide calculée par ICP
	gti320::Matrix4d m_initTM; // déformation initiale calculé au moment de charger le nuage de référence

	// Chemin vers le dossier contenant les fichiers OBJ
	std::string m_objFolder;

	double m_err; // erreur calculée entre le nuage source et le nuage de référence
};
