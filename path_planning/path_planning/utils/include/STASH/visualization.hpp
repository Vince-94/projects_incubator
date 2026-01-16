#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "graph.hpp"

#include <vector>
#include <string>

#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL_image.h>  // Optional



const int GRID_WIDTH = 30;
const int GRID_HEIGHT = 20;
const int CELL_SIZE = 30;  // Pixels per cell
const int WINDOW_WIDTH = GRID_WIDTH * CELL_SIZE;
const int WINDOW_HEIGHT = GRID_HEIGHT * CELL_SIZE;


// Simple grid: 0 = free, 1 = obstacle
// std::vector<std::vector<int>> grid(GRID_HEIGHT, std::vector<int>(GRID_WIDTH, 0));


class Visualization {
public:
    Visualization(const std::string name);

    ~Visualization();

    bool init();

    void visualizeGraph(const GraphPos graph_pos, const std::vector<int> path);


protected:
    void visualizePath(const std::vector<int> path, const GraphPos graph_pos);


private:
    std::string name_{};
    SDL_Window* window{};
    SDL_Renderer* renderer{};
    SDL_Texture* bg_texture{};
    TTF_Font* font{};
};

#endif  // VISUALIZATION_HPP