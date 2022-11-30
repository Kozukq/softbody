

#include "cgp/cgp.hpp" // Give access to the complete CGP library
#include <iostream>

// Library for ODE solving
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv2.h>

// Custom scene of this code
#include "scene.hpp"


// *************************** //
// Custom Scene defined in "scene.hpp"
// *************************** //

scene_structure scene;


// The rest of this code is a generic initialization and animation loop that can be applied to different scenes
// *************************** //
// Start of the program
// *************************** //

struct parameters {

    double c;
    double k;
    double M;
    double F;
};

window_structure standard_window_initialization(int width=0, int height=0);
void initialize_default_shaders();
int eqdiff(double t, const double y[], double f[], void* params);
int jacobian(double t, const double y[], double* dfdy, double dfdt[], void* params);

int main(int, char* argv[])
{
	std::cout << "Run " << argv[0] << std::endl;

	// ************************ //
	//     INITIALISATION
	// ************************ //
	
	// Standard Initialization of an OpenGL ready window
	scene.window = standard_window_initialization();



	// Initialize default shaders
	initialize_default_shaders();
	
	// Custom scene initialization
	std::cout << "Initialize data of the scene ..." << std::endl;
	scene.initialize();                                              
	std::cout << "Initialization finished\n" << std::endl;

	// Initialize ODE solver
	struct parameters parameters;
    parameters.c = 0.2;
    parameters.k = 2;
    parameters.M = 20;
    parameters.F = 5;
    gsl_odeiv2_system sys = {eqdiff, jacobian, 2, &parameters};
    gsl_odeiv2_driver* d = gsl_odeiv2_driver_alloc_y_new (&sys, gsl_odeiv2_step_rk1imp, 1e-6, 1e-6, 0.0);
    int ti;
    double t = 0.0;
    double y[2] = { 0.5, 0.0 }; // définir y[0] : position initiale, y[1] : vitesse initiale

	// ************************ //
	//     Animation Loop
	// ************************ //
	std::cout<<"Start animation loop ..."<<std::endl;
	timer_fps fps_record;
	fps_record.start();
	ti = 1;
	while (!glfwWindowShouldClose(scene.window.glfw_window))
	{
		scene.camera_projection.aspect_ratio = scene.window.aspect_ratio();
		scene.environment.camera_projection = scene.camera_projection.matrix();
		glViewport(0, 0, scene.window.width, scene.window.height);

		vec3 const& background_color = scene.environment.background_color;
		glPointSize(10.0f);
		glClearColor(background_color.x, background_color.y, background_color.z, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glClear(GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		float const time_interval = fps_record.update();
		if (fps_record.event) {
			std::string const title = "CGP Display - " + str(fps_record.fps) + " fps";
			glfwSetWindowTitle(scene.window.glfw_window, title.c_str());
		}

		imgui_create_frame();
		ImGui::Begin("GUI", NULL, ImGuiWindowFlags_AlwaysAutoResize);
		scene.inputs.mouse.on_gui = ImGui::GetIO().WantCaptureMouse;
		scene.inputs.time_interval = time_interval;

		// Physics
		gsl_odeiv2_driver_apply(d, &t, (double)ti, y);
		vec3 p2dir = vec3(0,0,y[0]) - vec3(0,0,3);
		p2dir /= norm(p2dir);
		// std::cout << y[1] << std::endl;
		scene.p2.model.translation += vec3(p2dir * y[1]);
		scene.line.initialize_data_on_gpu(mesh_primitive_line(vec3(0,0,2),scene.p2.model.translation));
		// scene.line.initialize_data_on_gpu(mesh_primitive_line(vec3(0,0,2),vec3(2,0,0.25) + scene.p2.model.translation));

		// Display the ImGUI interface (button, sliders, etc)
		scene.display_gui();

		// Handle camera behavior in standard frame
		scene.idle_frame();

		// Call the display of the scene
		scene.display_frame();

		// End of ImGui display and handle GLFW events
		ImGui::End();
		imgui_render_frame(scene.window.glfw_window); 
		glfwSwapBuffers(scene.window.glfw_window);
		glfwPollEvents();

		ti++;
	}
	std::cout << "\nAnimation loop stopped" << std::endl;
	
	// Cleanup
	cgp::imgui_cleanup();
	glfwDestroyWindow(scene.window.glfw_window);
	glfwTerminate();
	gsl_odeiv2_driver_free(d);

	return 0;
}

static void display_error_file_access();
void initialize_default_shaders()
{
	// Load default shader and initialize default frame
	// ***************************************************** //
	// Assume that the default shader file will be accessible in shaders/mesh/ and shaders/single_color/ directories
	if (check_file_exist("../shaders/mesh/vert.glsl") == false) {
		display_error_file_access();
		exit(1);
	}

	// Set standard mesh shader for mesh_drawable
	mesh_drawable::default_shader.load("../shaders/mesh/vert.glsl", "../shaders/mesh/frag.glsl");
	// Set default white texture
	image_structure const white_image = image_structure{ 1,1,image_color_type::rgba,{255,255,255,255} };
	mesh_drawable::default_texture.initialize_texture_2d_on_gpu(white_image);

	// Set standard uniform color for curve/segment_drawable
	curve_drawable::default_shader.load("../shaders/single_color/vert.glsl", "../shaders/single_color/frag.glsl");
}


static void display_error_file_access()
{
	std::cout << "[ERROR File Access] The default initialization from helper_common_scene tried to load the shader file shaders/mesh/vert.glsl but cannot not find it" << std::endl;
	std::cout << "  => In most situation, the problem is the following: Your executable is not run from the root directory of this scene, and the directory shaders/ is therefore not accessible. " << std::endl;
	std::cout << "  => To solve this problem, you may need to adjust your IDE settings (or your placement in command line) such that your executable is run from the parent directory of shaders/. Then run again the program. " << std::endl;
	std::cout << "\n\nThe program will now exit" << std::endl;
}


//Callback functions
void window_size_callback(GLFWwindow* window, int width, int height);
void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_click_callback(GLFWwindow* window, int button, int action, int mods);
void keyboard_callback(GLFWwindow* window, int key, int, int action, int mods);

// Standard initialization procedure
window_structure standard_window_initialization(int width_target, int height_target)
{
	// Create the window using GLFW
	// ***************************************************** //
	window_structure window;
	window.initialize(width_target, height_target);

	// Display information
	// ***************************************************** //

	// Display window size
	std::cout << "\nWindow (" << window.width << "px x " << window.height << "px) created" << std::endl;
	std::cout << "Monitor: " << glfwGetMonitorName(window.monitor) << " - Resolution (" << window.screen_resolution_width << "x" << window.screen_resolution_height << ")\n" << std::endl;

	// Display debug information on command line
	std::cout << "OpenGL Information:" << std::endl;
	std::cout << cgp::opengl_info_display() << std::endl;

	// Initialize ImGUI
	// ***************************************************** //
	cgp::imgui_init(window.glfw_window);

	// Set the callback functions for the inputs
	glfwSetMouseButtonCallback(window.glfw_window, mouse_click_callback); // Event called when a button of the mouse is clicked/released
	glfwSetCursorPosCallback(window.glfw_window, mouse_move_callback);    // Event called when the mouse is moved
	glfwSetWindowSizeCallback(window.glfw_window, window_size_callback);  // Event called when the window is rescaled        
	glfwSetKeyCallback(window.glfw_window, keyboard_callback);            // Event called when a keyboard touch is pressed/released


	return window;
}

int eqdiff(double t, const double y[], double f[], void* params) {

	(void)(t);

    struct parameters parameters = *(struct parameters*)params;

    double c = parameters.c;
    double k = parameters.k;
    double M = parameters.M;
    double F = parameters.F;

    f[0] = y[1];
    f[1] = (F - c * y[1] - k * y[0]) / M;

    return GSL_SUCCESS;
}

int jacobian(double t, const double y[], double* dfdy, double dfdt[], void* params) {

	(void)(t);
	(void)(y);

    struct parameters parameters = *(struct parameters*)params;

    double c = parameters.c;
    double k = parameters.k;
    double M = parameters.M;

    gsl_matrix_view dfdy_mat = gsl_matrix_view_array (dfdy, 2, 2);
    
    gsl_matrix* m = &dfdy_mat.matrix;
    gsl_matrix_set(m, 0, 0, 0.0);
    gsl_matrix_set(m, 0, 1, 1.0);
    gsl_matrix_set(m, 1, 0, -k / M);
    gsl_matrix_set(m, 1, 1, -c / M);

    dfdt[0] = 0.0;
    dfdt[1] = 0.0;
    
    return GSL_SUCCESS;
}

// This function is called everytime the window is resized
void window_size_callback(GLFWwindow* , int width, int height)
{
	scene.window.width = width;
	scene.window.height = height;
}

// This function is called everytime the mouse is moved
void mouse_move_callback(GLFWwindow* /*window*/, double xpos, double ypos)
{
	vec2 const pos_relative = scene.window.convert_pixel_to_relative_coordinates({ xpos, ypos });
	scene.inputs.mouse.position.update(pos_relative);
	scene.mouse_move_event();
}

// This function is called everytime a mouse button is clicked/released
void mouse_click_callback(GLFWwindow* /*window*/, int button, int action, int /*mods*/)
{
	scene.inputs.mouse.click.update_from_glfw_click(button, action);
	scene.mouse_click_event();

}

// This function is called everytime a keyboard touch is pressed/released
void keyboard_callback(GLFWwindow* /*window*/, int key, int , int action, int /*mods*/)
{
	scene.inputs.keyboard.update_from_glfw_key(key, action);
	scene.keyboard_event();

	// Press 'F' for full screen mode
	if (key == GLFW_KEY_F && action == GLFW_PRESS && scene.inputs.keyboard.shift) {
		scene.window.is_full_screen = !scene.window.is_full_screen;
		if (scene.window.is_full_screen)
			scene.window.set_full_screen();
		else
			scene.window.set_windowed_screen();
	}
	// Press 'V' for camera frame/view matrix debug
	if (key == GLFW_KEY_V && action == GLFW_PRESS && scene.inputs.keyboard.shift) {
		auto const camera_model = scene.camera_control.camera_model;
		std::cout << "\nDebug camera (position = "<<str(camera_model.position())<<"):\n" << std::endl;
		std::cout << "  Frame matrix:" << std::endl;
		std::cout << str_pretty(camera_model.matrix_frame()) << std::endl;
		std::cout << "  View matrix:" << std::endl;
		std::cout << str_pretty(camera_model.matrix_view()) << std::endl;
		
	}



}





