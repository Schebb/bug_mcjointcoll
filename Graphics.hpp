
#ifndef __MCPLANE_GRAPHICS_HPP__
# define __MCPLANE_GRAPHICS_HPP__

# define GLEW_STATIC
# include <memory>
# include <GL/glew.h>
# include <SDL2/SDL_opengl.h>
# include <GL/glu.h>
# include <GL/gl.h>
# include <glm/glm.hpp>
# include <glm/gtc/type_ptr.hpp>	
# include <glm/gtx/string_cast.hpp>	
# include <glm/gtc/matrix_transform.hpp>	
# include <glm/glm.hpp>
# include <SDL2/SDL.h>


using namespace glm;
using Color = vec3;


struct SDLDeleter 
{
	void operator()(SDL_Surface*  ptr) { if (ptr) SDL_FreeSurface(ptr); }
	void operator()(SDL_Texture*  ptr) { if (ptr) SDL_DestroyTexture(ptr); }
	void operator()(SDL_Renderer* ptr) { if (ptr) SDL_DestroyRenderer(ptr); }
	void operator()(SDL_Window*   ptr) { if (ptr) SDL_DestroyWindow(ptr); }
	void operator()(SDL_RWops*    ptr) { if (ptr) SDL_RWclose(ptr); }
};

template<class T, class D = std::default_delete<T>>
struct shared_ptr_with_deleter : public std::shared_ptr<T>
{
	explicit shared_ptr_with_deleter(T* t = nullptr)
		: std::shared_ptr<T>(t, D()) {}

	void reset(T* t = nullptr) {
		std::shared_ptr<T>::reset(t, D());
	}
};

using SDLSurfacePtr   = shared_ptr_with_deleter<SDL_Surface, SDLDeleter>;
using SDLTexturePtr   = shared_ptr_with_deleter<SDL_Texture, SDLDeleter>;
using SDLRendererPtr  = shared_ptr_with_deleter<SDL_Renderer, SDLDeleter>;
using SDLWindowPtr    = shared_ptr_with_deleter<SDL_Window, SDLDeleter>;

using SDLSurfaceUPtr    = std::unique_ptr<SDL_Surface, SDLDeleter>;
using SDLTextureUPtr    = std::unique_ptr<SDL_Texture, SDLDeleter>;
using SDLRendererUPtr   = std::unique_ptr<SDL_Renderer, SDLDeleter>;
using SDLWindowUPtr     = std::unique_ptr<SDL_Window, SDLDeleter>;

inline	void glUniform(GLint location, const glm::mat4 &mat) { glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(mat)); }
inline	void glUniform(GLint location, const glm::mat3 &mat) { glUniformMatrix3fv(location, 1, GL_FALSE, glm::value_ptr(mat)); }
inline	void glUniform(GLint location, const glm::mat2 &mat) { glUniformMatrix2fv(location, 1, GL_FALSE, glm::value_ptr(mat)); }
inline	void glUniform(GLint location, const glm::vec4 &vec) { glUniform4f(location, vec.x, vec.y, vec.z, vec.w); }
inline	void glUniform(GLint location, const glm::vec3 &vec) { glUniform3f(location, vec.x, vec.y, vec.z); }
inline	void glUniform(GLint location, const glm::vec2 &vec) { glUniform2f(location, vec.x, vec.y); }
inline	void glUniform(GLint location, GLfloat f) { glUniform1f(location, f); }
inline	void glUniform(GLint location, GLint i) { glUniform1i(location, i); }
inline	void glUniform(GLint location, GLuint i) { glUniform1ui(location, i); }

///
/// Manage everything related to Graphics.
///
class Graphics
{
	public:
		bool 	init( unsigned width, unsigned height );
		void 	deinit( void );

		void 	clear( void );
		void 	drawBox( const mat4& model, const Color& color );
		void 	refresh( void );

	private:
		SDLWindowUPtr 	_win = nullptr;
		SDL_GLContext 	_context;

		GLuint 			_boxVAO = 0;
		GLuint 			_boxVBO = 0;
		GLuint  		_fragId     = 0;  ///< fragment shader id
		GLuint  		_vertId     = 0;  ///< vertex shader id
		GLuint  		_programId  = 0;  ///< program id (attaching both fragment and vertex shaders)

		GLint 			_unifProj = 0;
		GLint 			_unifView = 0;
		GLint 			_unifModel = 0;
		GLint 			_unifColor = 0;

		mat4 			_proj;
		mat4 			_view;

};


#endif // __MCPLANE_GRAPHICS_HPP__
