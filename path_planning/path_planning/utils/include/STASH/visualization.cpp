#include "visualization.hpp"

#include <iostream>


Visualization::Visualization(const std::string name) {
    name_ = name;
}


Visualization::~Visualization() {
    TTF_CloseFont(font);
    if (bg_texture) SDL_DestroyTexture(bg_texture);
    IMG_Quit();
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}


bool Visualization::init() {
    // Initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Initialize TTF
    TTF_Init();

    // Initialize IMG
    IMG_Init(IMG_INIT_PNG);  // Optional

    window = SDL_CreateWindow(
        name_.data(),
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN
    );
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return false;
    }

    font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24);  // TODO Replace with actual font path

    return true;
}


void Visualization::visualizeGraph(const GraphPos graph_pos, const std::vector<int> path) {
    // Define node positions (hardcoded for 3x3 layout)
    auto graph = graph_pos.graph;
    auto positions = graph_pos.positions;
    const int NODE_RADIUS = 20;

    // Optional: Load background image
    // bg_texture = IMG_LoadTexture(renderer, "path/to/background.png");

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
        }

        SDL_SetRenderDrawColor(renderer, 20, 20, 40, 255);  // Dark background
        SDL_RenderClear(renderer);

        // Optional: Draw background
        if (bg_texture) {
            SDL_RenderCopy(renderer, bg_texture, NULL, NULL);
        }

        // Draw edges (using SDL2_gfx for thickness)
        for (size_t i = 0; i < graph.size(); ++i) {
            for (int j : graph[i]) {
                if (j > i) {  // Avoid duplicate lines in undirected graph
                    thickLineRGBA(renderer,
                                positions[i].x, positions[i].y,
                                positions[j].x, positions[j].y,
                                4,  // Thickness
                                255, 255, 255, 255);  // White
                }
            }
        }

        // Draw nodes (circles with SDL2_gfx)
        for (size_t i = 0; i < positions.size(); ++i) {
            filledCircleRGBA(renderer, positions[i].x, positions[i].y, NODE_RADIUS,
                            100, 100, 255, 255);  // Blue fill
            circleRGBA(renderer, positions[i].x, positions[i].y, NODE_RADIUS,
                    255, 255, 255, 255);  // White outline
        }

        // TODO Draw path
        visualizePath(path, graph_pos);

        // Draw labels (text with SDL2_ttf)
        SDL_Color text_color = {255, 255, 255, 255};  // White
        for (size_t i = 0; i < positions.size(); ++i) {
            std::string label = std::to_string(i);
            SDL_Surface* text_surface = TTF_RenderText_Solid(font, label.c_str(), text_color);
            SDL_Texture* text_texture = SDL_CreateTextureFromSurface(renderer, text_surface);

            int tex_w = 0, tex_h = 0;
            SDL_QueryTexture(text_texture, NULL, NULL, &tex_w, &tex_h);

            SDL_Rect dst_rect = {positions[i].x - tex_w / 2,
                                positions[i].y - tex_h / 2,
                                tex_w, tex_h};
            SDL_RenderCopy(renderer, text_texture, NULL, &dst_rect);

            SDL_FreeSurface(text_surface);
            SDL_DestroyTexture(text_texture);
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(16);  // ~60 FPS
    }
}


void Visualization::visualizePath(const std::vector<int> path, const GraphPos graph_pos) {
    if (path.size() < 2) return;

    auto positions = graph_pos.positions;

    // Draw path edges
    for (size_t i = 1; i < path.size(); ++i) {
        int u = path[i - 1];
        int v = path[i];

        thickLineRGBA(
            renderer,
            positions[u].x, positions[u].y,
            positions[v].x, positions[v].y,
            6,                  // thicker than normal edges
            255, 50, 50, 255     // red
        );
    }

    // Draw path nodes
    for (int node : path) {
        filledCircleRGBA(
            renderer,
            positions[node].x,
            positions[node].y,
            24,                 // slightly larger
            255, 80, 80, 255
        );
        circleRGBA(
            renderer,
            positions[node].x,
            positions[node].y,
            24,
            255, 255, 255, 255
        );
    }
}
