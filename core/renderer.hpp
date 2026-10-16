#ifndef __RENDERER_H
#define __RENDERER_H
#include <string>

using namespace std;

// 1360x768: the window/logical render resolution - referenced throughout
// Game.cpp for camera centring, zoom bounds, and edge-scroll thresholds.
#define WINDOW_WIDTH     1360
#define WINDOW_HEIGHT    768

// FILE: SDL_Window/SDL_Renderer singleton.
// FITS IN: every draw call in the codebase goes through
// Renderer::Instance()->getRenderer(). IMPORTANT INVARIANT: call
// initRenderer() once (initSDL does this) before any getRenderer()/getWindow().
class Renderer
{
private:
	Renderer(){}; // Singleton private constructor
	static Renderer * s_pInstance;
	static SDL_Window *window;
    static SDL_Renderer *renderer;

public:
	~Renderer(){ };
	static Renderer* Instance();
    void initRenderer(string windowCaption);
    SDL_Renderer *getRenderer();
    SDL_Window *getWindow();
    void closeRenderer();
};

#endif // __RENDERER_H
