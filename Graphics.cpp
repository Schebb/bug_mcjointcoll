
#include <iostream>
#include <cassert>
#include "Graphics.hpp"

#define SHADER_ATTRIB_OUT 		"OutColor"
#define SHADER_ATTRIB_POSITION 	"Position"
#define SHADER_ATTRIB_NORMAL 	"Normal"

const char* vertexShader = R"str(
#version 330 core

uniform mat4 proj;
uniform mat4 view;
uniform mat4 model;

layout (location = 0) in vec3 Position;
layout (location = 1) in vec3 Normal;

out VS_OUT
{
	float light;
} vs_out;

void main() {
	// direction of the sun
	vec3 sunDir = normalize(vec3(0.5, 1, 0.25));

	mat4 rot = model;
	rot[3][0] = 0;
	rot[3][1] = 0;
	rot[3][2] = 0;
	vec3 N = normalize((rot*vec4(Normal, 1.0)).xyz);
	vs_out.light = max(dot(N, sunDir), 0.0);
	gl_Position = proj * view * model * vec4(Position, 1.0);
}

)str";

const char* fragShader = R"str(
#version 330 core

uniform vec3 color;

layout (location = 0) out vec4 OutColor;

in VS_OUT
{
	float light;
} fs_in;

void main() {
	OutColor = vec4(color * fs_in.light, 1.0);
}

)str";

bool 	loadShader( GLuint shaderId, const char* src, std::string& outputlog )
{
	char buffer[512];
	outputlog.clear();

	glShaderSource(shaderId, 1, &src, NULL);
	glCompileShader(shaderId);

	glGetShaderInfoLog(shaderId, 512, NULL, buffer);
	outputlog = std::string(buffer);

	GLint status = GL_FALSE;
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &status);
	return (status == GL_TRUE);
}

bool 	Graphics::init( unsigned width, unsigned height )
{
	// Use OpenGL 3.1 core
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 3 );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 3 );
	SDL_GL_SetAttribute( SDL_GL_DOUBLEBUFFER, 1 );
	SDL_GL_SetAttribute( SDL_GL_ACCELERATED_VISUAL, 1 );

	SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");

	// Enable multisampling for a nice antialiased effect 
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

	// Create window
	_win.reset(
			SDL_CreateWindow( "mctest", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
				width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN ));

	if (!_win)
	{
		std::cout << "Window could not be created! SDL Error: " << SDL_GetError();
		return false;
	}

	// Create context
	_context = SDL_GL_CreateContext(_win.get());
	if (!_context)
	{
		std::cout << "OpenGL context could not be created! SDL Error: " << SDL_GetError();
		return false;
	}

	// Enable VSync
	if( SDL_GL_SetSwapInterval(1) < 0 )
		std::cout << "unable to set VSync! SDL Error: " << SDL_GetError();

	assert(glGetError() == GL_NO_ERROR);

	std::cout << "Opengl Version: " << glGetString(GL_VERSION) << std::endl;
	std::cout << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;

	//Initialize GLEW
	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();
	if( glewError != GLEW_OK )
		std::cout << "impossible to initialize GLEW! " << glewGetErrorString( glewError );

	// ignore gl's errors after glewExperimentat init:
	//https://www.opengl.org/wiki/OpenGL_Loading_Library
	glGetError();


	//Load shaders
	_fragId = glCreateShader(GL_FRAGMENT_SHADER);
	_vertId = glCreateShader(GL_VERTEX_SHADER);
	_programId = glCreateProgram();

	std::string outputlog;
	if (loadShader(_vertId, vertexShader, outputlog) == false
			|| loadShader(_fragId, fragShader, outputlog) == false)
	{
		std::cout << "error while compiling shaders: \n" << outputlog << std::endl;
		return false;
	}

	glAttachShader(_programId, _vertId);
	glAttachShader(_programId, _fragId);

	
	glBindFragDataLocation(_programId, 0, SHADER_ATTRIB_OUT);
	glBindAttribLocation(_programId, 0, SHADER_ATTRIB_POSITION);
	glBindAttribLocation(_programId, 1, SHADER_ATTRIB_NORMAL);

	glLinkProgram(_programId);

	assert(glGetError() == GL_NO_ERROR);

	GLint programSuccess = GL_TRUE;
	glGetProgramiv(_programId, GL_LINK_STATUS, &programSuccess);
	if ( programSuccess != GL_TRUE)
	{
		std::cout << "failed to link shader program";
		return false;
	}

	glUseProgram(_programId);

	_unifProj = glGetUniformLocation(_programId, "proj");
	_unifView = glGetUniformLocation(_programId, "view");
	_unifModel = glGetUniformLocation(_programId, "model");
	_unifColor = glGetUniformLocation(_programId, "color");

	// Generate a Box
	glGenVertexArrays(1, &_boxVAO);
	glBindVertexArray(_boxVAO);


	// Create a Vertex Buffer Object and copy the vertex data to it
	glGenBuffers(1, &_boxVBO);
	glBindBuffer(GL_ARRAY_BUFFER, _boxVBO);

	GLfloat vertices[] = {
		-0.5f,	-0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	
		0.5f,	-0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	
		0.5f,	 0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	
		0.5f,	 0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	
		-0.5f,	 0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	
		-0.5f,	-0.5f,	-0.5f,	0.0f,	0.0f,	-1.0f,	

		-0.5f,	-0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	
		0.5f,	-0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	
		0.5f,	 0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	
		0.5f,	 0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	
		-0.5f,	 0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	
		-0.5f,	-0.5f,	 0.5f,	0.0f,	0.0f,	1.0f,	

		-0.5f,	 0.5f,	 0.5f,	-1.0f,	0.0f,	0.0f,	
		-0.5f,	 0.5f,	-0.5f,	-1.0f,	0.0f,	0.0f,	
		-0.5f,	-0.5f,	-0.5f,	-1.0f,	0.0f,	0.0f,	
		-0.5f,	-0.5f,	-0.5f,	-1.0f,	0.0f,	0.0f,	
		-0.5f,	-0.5f,	 0.5f,	-1.0f,	0.0f,	0.0f,	
		-0.5f,	 0.5f,	 0.5f,	-1.0f,	0.0f,	0.0f,	

		0.5f,	 0.5f,	 0.5f,	1.0f,	0.0f,	0.0f,	
		0.5f,	 0.5f,	-0.5f,	1.0f,	0.0f,	0.0f,	
		0.5f,	-0.5f,	-0.5f,	1.0f,	0.0f,	0.0f,	
		0.5f,	-0.5f,	-0.5f,	1.0f,	0.0f,	0.0f,	
		0.5f,	-0.5f,	 0.5f,	1.0f,	0.0f,	0.0f,	
		0.5f,	 0.5f,	 0.5f,	1.0f,	0.0f,	0.0f,	

		-0.5f,	-0.5f,	-0.5f,	0.0f,	-1.0f,	0.0f,	
		0.5f,	-0.5f,	-0.5f,	0.0f,	-1.0f,	0.0f,	
		0.5f,	-0.5f,	 0.5f,	0.0f,	-1.0f,	0.0f,	
		0.5f,	-0.5f,	 0.5f,	0.0f,	-1.0f,	0.0f,	
		-0.5f,	-0.5f,	 0.5f,	0.0f,	-1.0f,	0.0f,	
		-0.5f,	-0.5f,	-0.5f,	0.0f,	-1.0f,	0.0f,	

		-0.5f,	 0.5f,	-0.5f,	0.0f,	1.0f,	0.0f,	
		0.5f,	 0.5f,	-0.5f,	0.0f,	1.0f,	0.0f,	
		0.5f,	 0.5f,	 0.5f,	0.0f,	1.0f,	0.0f,	
		0.5f,	 0.5f,	 0.5f,	0.0f,	1.0f,	0.0f,	
		-0.5f,	 0.5f,	 0.5f,	0.0f,	1.0f,	0.0f,	
		-0.5f,	 0.5f,	-0.5f,	0.0f,	1.0f,	0.0f
	};

	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


	
	// Specify the layout of the vertex data
	glEnableVertexAttribArray(0/*SHADER_ATTRIB_POSITION*/);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), 0);

	glEnableVertexAttribArray(1/*SHADER_ATTRIB_NORMAL*/);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (void*)(3 * sizeof(GLfloat)));

	// Application Settings
	mat4 	_proj = perspective( 3.14f/3.f, (float)width/(float)height, 0.1f, 1000.f);
	mat4 	_view = lookAt(vec3(5, 6, 5)*3.f, vec3(0.f, 0.f, -30.f), vec3(0.f, 1.f, 0.f));
	glUniform(_unifProj, _proj);
	glUniform(_unifView, _view);

	glDepthMask( GL_TRUE );
	glDepthFunc( GL_LESS );
	glEnable(GL_DEPTH_TEST);

	return true;
}

void 	Graphics::deinit( void )
{
	if (_fragId) glDeleteShader(_fragId);
	if (_vertId) glDeleteShader(_vertId);
	if (_programId) glDeleteProgram(_programId);
	glDeleteBuffers(1, &_boxVBO);
	glDeleteVertexArrays(1, &_boxVAO);
	_win.reset();
}

void 	Graphics::clear( void )
{
	const GLfloat  clearColor = 0.7f;
	glClearColor(clearColor, clearColor, clearColor, 0.f);
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
}

void 	Graphics::drawBox( const mat4& model, const Color& color )
{
	glUniform(_unifModel, model);
	glUniform(_unifColor, color);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

void 	Graphics::refresh( void )
{
	SDL_GL_SwapWindow(_win.get());
}

