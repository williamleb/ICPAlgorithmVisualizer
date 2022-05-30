/**
 * @file IcpApplication.cpp
 *
 * @brief Partie applicative du Laboratoire #2 :
 *         - Implémentation de l'algorithme ICP
 *         - Gestion de l'interface graphique
 *
 * Nom: William Lebel
 * Email : william.lebel.1@ens.etsmtl.ca
 *
 */

#include "IcpApplication.h"
#include "IcpGLCanvas.h"

#include <nanogui/window.h>
#include <nanogui/formhelper.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/button.h>
#include <nanogui/progressbar.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/opengl.h>
#include <nanogui/glutil.h>

#include <fstream>
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace nanogui;

namespace
{
	/**
	 * Générateur de nombres pseudo-aléatoire.
	 *
	 * @param min borne inférieure
	 * @param max borne supérieure
	 * @return un nombre aléatoire val tel que _min <= val <= _max
	 */
	template <typename Scalar>
	static inline Scalar rand(Scalar min, Scalar max)
	{
		static std::random_device rdev;
		static std::default_random_engine re(rdev());

		// Utilisation d'une distribution uniforme (recommandé depuis C++11)
		typedef typename std::conditional<
			std::is_floating_point<Scalar>::value,
			std::uniform_real_distribution<Scalar>,
			std::uniform_int_distribution<Scalar>>::type dist_type;
		dist_type uni(min, max);
		return static_cast<Scalar>(uni(re));
	}

	/**
	 * Génère le nuage de points generated à partir du nuage source en lui appliquant une
	 * transformation rigide aléatoire.
	 *
	 * Soit a_i le i-ème point de source et b_i le i-ème point de generated.
	 * b_i = R*a_i + t
	 * où R est une matrice de rotation et t un vecteur de translation.
	 *
	 * @param source nuage de points de référence (input)
	 * @param generated nuage de points perturbé (output)
	 */
	static inline void copyAndPerturbPoints(const gti320::Points3d& source, gti320::Points3d& generated)
	{
		static const double maxRot = M_PI / 8.0;
		static const double maxTra = 2.0;

		const gti320::Matrix3d R = gti320::makeRotation(rand(-maxRot, maxRot), rand(-maxRot, maxRot),
		                                                rand(-maxRot, maxRot));
		gti320::Vector3d t;
		t(0) = rand(-maxTra, maxTra);
		t(1) = rand(-maxTra, maxTra);
		t(2) = rand(-maxTra, maxTra);

		const int ncols = source.cols();
		generated.resize(3, ncols);
		for (int i = 0; i < ncols; ++i)
		{
			gti320::Vector3d p;
			p(0) = source(0, i);
			p(1) = source(1, i);
			p(2) = source(2, i);
			const gti320::Vector3d pp = R * p + t;
			generated(0, i) = pp(0);
			generated(1, i) = pp(1);
			generated(2, i) = pp(2);
		}
	}

	/**
	 * Charge un fichier OBJ.
	 *
	 * Tous les points du fichier .obj sont enregistrés dans le nuage de points
	 * _verts.
	 *
	 * @param filename nom du fichier à charger (input)
	 * @param verts   nuage de points (output)
	 * @return true si la lecture du fichier a fonctionné, false sinon.
	 */
	bool loadFile(const std::string& filename, gti320::Points3d& verts)
	{
		// Open the input file
		std::ifstream fs(filename, std::ifstream::in);
		if (!fs.is_open())
		{
			std::cout << "Error: Failed to open file " << filename << " for reading!" << std::endl;
			return false;
		}

		// Read file - first pass count the vertices
		std::string line;
		int vertCount = 0;
		while (std::getline(fs, line)) { if (line[0] == 'v' && line[1] == ' ') ++vertCount; }

		verts.resize(3, vertCount);

		// Read file - first pass count the vertices
		int col = 0;
		fs.clear();
		fs.seekg(std::ios::beg);
		while (std::getline(fs, line))
		{
			if (line[0] == 'v' && line[1] == ' ')
			{
				// Vertex! Add it to the list.
				float x, y, z;
				std::stringstream ss(line.substr(2));
				ss >> x >> y >> z;

				verts(0, col) = x;
				verts(1, col) = y;
				verts(2, col) = z;
				++col;
			}
		}

		fs.close();

		return true;
	}
}

/**
 * Constructeur par défaut
 */
IcpApplication::IcpApplication(const char* objFolder)
: nanogui::Screen(Eigen::Vector2i(1280, 720), "GTI320 Labo 02", true, false, 8, 8, 24, 8, 0, 4, 1), m_objFolder(objFolder)
{
	loadModel("bunny.obj"); // modèle par défaut

	m_err = 0.0;

	///////////////////////////////////////////////////////////////
	//
	// Construction du GUI
	//
	m_window = new Window(this, "Nuages de points");
	m_window->setPosition(Vector2i(8, 8));
	m_window->setLayout(new GroupLayout());

	m_canvas = new IcpGLCanvas(this);
	m_canvas->setBackgroundColor({255, 255, 255, 255});
	m_canvas->setSize({1080, 600});


	Window* controls = new Window(this, "Controls");
	controls->setPosition(Vector2i(1020, 10));
	controls->setLayout(new GroupLayout());

	Widget* tools = new Widget(controls);
	tools->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Middle, 0, 5));

	const auto translateMinMax = std::make_pair<float, float>(-10.0f, 10.0f);
	const auto rotateMinMax = std::make_pair<float, float>(-180.0f, 180.0f);

	// Slider de la translation en X
	m_panelTranslateX = new Widget(tools);
	m_panelTranslateX->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelTranslateX = new Label(m_panelTranslateX, "Translate X : ");
	m_sliderTranslateX = new Slider(m_panelTranslateX);
	m_sliderTranslateX->setRange(translateMinMax);
	m_textboxTranslateX = new TextBox(m_panelTranslateX);
	m_sliderTranslateX->setCallback([this](float value)
	{
		m_translation(0) = value;
		onSlidersChanged();
	});

	// Slider de la translation en Y
	m_panelTranslateY = new Widget(tools);
	m_panelTranslateY->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelTranslateY = new Label(m_panelTranslateY, "Translate Y : ");
	m_sliderTranslateY = new Slider(m_panelTranslateY);
	m_sliderTranslateY->setRange(translateMinMax);
	m_textboxTranslateY = new TextBox(m_panelTranslateY);
	m_sliderTranslateY->setCallback([this](float value)
	{
		m_translation(1) = value;
		onSlidersChanged();
	});

	// Slider de la translation en Z
	m_panelTranslateZ = new Widget(tools);
	m_panelTranslateZ->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelTranslateZ = new Label(m_panelTranslateZ, "Translate Z : ");
	m_sliderTranslateZ = new Slider(m_panelTranslateZ);
	m_sliderTranslateZ->setRange(translateMinMax);
	m_textboxTranslateZ = new TextBox(m_panelTranslateZ);
	m_sliderTranslateZ->setCallback([this](float value)
	{
		m_translation(2) = value;
		onSlidersChanged();
	});

	// Slider de la rotation en X
	m_panelRotateX = new Widget(tools);
	m_panelRotateX->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelRotateX = new Label(m_panelRotateX, "Rotate X : ");
	m_sliderRotateX = new Slider(m_panelRotateX);
	m_sliderRotateX->setRange(rotateMinMax);
	m_textboxRotateX = new TextBox(m_panelRotateX);
	m_sliderRotateX->setCallback([this](float value)
	{
		m_rotation(0) = value;
		onSlidersChanged();
	});

	// Slider de la rotation en Y
	m_panelRotateY = new Widget(tools);
	m_panelRotateY->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelRotateY = new Label(m_panelRotateY, "Rotate Y : ");
	m_sliderRotateY = new Slider(m_panelRotateY);
	m_sliderRotateY->setRange(rotateMinMax);
	m_textboxRotateY = new TextBox(m_panelRotateY);
	m_sliderRotateY->setCallback([this](float value)
	{
		m_rotation(1) = value;
		onSlidersChanged();
	});

	// Slider de la rotation en Z
	m_panelRotateZ = new Widget(tools);
	m_panelRotateZ->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelRotateZ = new Label(m_panelRotateZ, "Rotate Z : ");
	m_sliderRotateZ = new Slider(m_panelRotateZ);
	m_sliderRotateZ->setRange(rotateMinMax);
	m_textboxRotateZ = new TextBox(m_panelRotateZ);
	m_sliderRotateZ->setCallback([this](float value)
	{
		m_rotation(2) = value;
		onSlidersChanged();
	});

	// Bouton pour effectuer une itération de l'ICP
	Button* icpButton = new Button(tools, "ICP (1 iteration)");
	icpButton->setCallback([this]
	{
		stepIcp();
	});

	// Bouton reset
	Button* resetButton = new Button(tools, "Reset");
	resetButton->setCallback([this]
	{
		reset();
	});

	// Bouton pour charger le modèle `cone`
	Button* loadConeButton = new Button(tools, "Load cone");
	loadConeButton->setCallback([this]
	{
		loadModel("cone.obj");
	});

	// Bouton pour charger le modèle `bunny`
	Button* loadBunnyButton = new Button(tools, "Load bunny");
	loadBunnyButton->setCallback([this]
	{
		loadModel("bunny.obj");
	});

	// Bouton pour charger le modèle `sphere`
	Button* loadSphereButton = new Button(tools, "Load sphere");
	loadSphereButton->setCallback([this]
	{
		loadModel("sphere.obj");
	});

	// Bouton pour charger le modèle `droite`
	Button* loadLineButton = new Button(tools, "Load line");
	loadLineButton->setCallback([this]
	{
		loadModel("droite.obj");
	});

	// Affichage de l'erreur entre les deux nuages de points
	m_panelError = new Widget(tools);
	m_panelError->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 20));
	m_labelError = new Label(m_panelError, "Error :");
	m_textboxError = new TextBox(m_panelError);
	m_textboxError->setFixedWidth(160);

	performLayout();
	reset();
}

/**
 * Callback déclanché par le clavier
 */
bool IcpApplication::keyboardEvent(int key, int scancode, int action, int modifiers)
{
	if (Screen::keyboardEvent(key, scancode, action, modifiers))
		return true;
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		setVisible(false);
		return true;
	}
	return false;
}


/**
 * Gestion des sliders lorsqu'un de ceux-ci est modifié
 */
void IcpApplication::onSlidersChanged()
{
	// Lecture des valeurs sur le GUI
	static char buf[32];
	snprintf(buf, sizeof(buf), "%3.3f", m_translation(0));
	m_textboxTranslateX->setValue(buf);
	snprintf(buf, sizeof(buf), "%3.3f", m_translation(1));
	m_textboxTranslateY->setValue(buf);
	snprintf(buf, sizeof(buf), "%3.3f", m_translation(2));
	m_textboxTranslateZ->setValue(buf);
	snprintf(buf, sizeof(buf), "%3.3f", m_rotation(0));
	m_textboxRotateX->setValue(buf);
	snprintf(buf, sizeof(buf), "%3.3f", m_rotation(1));
	m_textboxRotateY->setValue(buf);
	snprintf(buf, sizeof(buf), "%3.3f", m_rotation(2));
	m_textboxRotateZ->setValue(buf);

	// Construction de la matrice de transformation rigide correspondant à la
	// transformation donnée par le GUI.
	static const double deg2rad = M_PI / 180.0;
	const gti320::Matrix3d R = gti320::makeRotation(deg2rad * m_rotation(0), deg2rad * m_rotation(1),
	                                                deg2rad * m_rotation(2));
	m_initTM.block(0, 0, 3, 3) = R;
	m_initTM(0, 3) = m_translation(0);
	m_initTM(1, 3) = m_translation(1);
	m_initTM(2, 3) = m_translation(2);

	// MAJ de l'erreur affichée
	computeError();
}

/**
 * Une itération de l'ICP
 */
void IcpApplication::stepIcp()
{
	const int numberOfPoints = m_refPoints.cols();

	gti320::Points3d destination(3, numberOfPoints);
	gti320::Points3d source(3, numberOfPoints);

	for (int j = 0; j < numberOfPoints; ++j)
	{
		// Étape 1.1 - p = m_srcTM * m_initTM * m_sourcePoints(:,i)
		gti320::Vector3d sourcePoint = m_sourcePoints.col(j);
		sourcePoint = m_srcTM * m_initTM * sourcePoint;

		// Étape 1.2 -On trouve le point le plus près
		int nearestNeighborIndex = gti320::Icp::nearestNeighbor(sourcePoint, m_refPoints);

		// Étape 1.3 -B[:,i] = p
		source(0, j) = sourcePoint.x();
		source(1, j) = sourcePoint.y();
		source(2, j) = sourcePoint.z();

		// Étape 1.4 -A[:,i] = m_refPoints[:,k]
		destination(0, j) = m_refPoints(0, nearestNeighborIndex);
		destination(1, j) = m_refPoints(1, nearestNeighborIndex);
		destination(2, j) = m_refPoints(2, nearestNeighborIndex);
	}

	// Étape 2 - On calcul la meilleure transformation pour que le nuage de points source se déplace vers destination
	gti320::Matrix4d bestFitTransform = gti320::Icp::bestFitTransform(destination, source);
	m_srcTM = bestFitTransform * m_srcTM;

	computeError();
}

void IcpApplication::reset()
{
	m_srcTM.setIdentity();
	m_initTM.setIdentity();
	m_translation.setZero();
	m_rotation.setZero();

	computeError();

	m_sliderRotateX->setValue(0.0);
	m_sliderRotateY->setValue(0.0);
	m_sliderRotateZ->setValue(0.0);
	m_sliderTranslateX->setValue(0.0);
	m_sliderTranslateY->setValue(0.0);
	m_sliderTranslateZ->setValue(0.0);
	onSlidersChanged(); // les affectation ne déclanchent pas le callback, on le fait manuellement
}

void IcpApplication::computeError()
{
	const int npoints = m_refPoints.cols();

	// Contient les points transformés
	gti320::Points3d srcPointsTM(3, npoints);

	// Calcule la position réelle de chacun des points du nuage source en leur
	// appliquant la transformation initiale (donnée par le GUI) et la
	// transformation source (calculé par ICP).
	gti320::Vector3d p;
	for (int i = 0; i < npoints; ++i)
	{
		p(0) = m_sourcePoints(0, i);
		p(1) = m_sourcePoints(1, i);
		p(2) = m_sourcePoints(2, i);
		p = m_srcTM * m_initTM * p;
		srcPointsTM(0, i) = p(0);
		srcPointsTM(1, i) = p(1);
		srcPointsTM(2, i) = p(2);
	}

	m_err = gti320::Icp::error(m_refPoints, srcPointsTM);

	// Affichage de l'erreur
	//
	static char buf[32];
	snprintf(buf, sizeof(buf), "%9.3f", m_err);
	m_textboxError->setValue(buf);
}


void IcpApplication::loadModel(const std::string& filename)
{
	loadFile(m_objFolder + filename, m_refPoints);
	copyAndPerturbPoints(m_refPoints, m_sourcePoints);
}
