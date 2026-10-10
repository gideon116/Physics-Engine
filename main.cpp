#include <iostream>
#include "player.h"
#include "physics.h"
#include <SFML/Graphics.hpp>


int main()
{
    int frame = 0;
    int delay = 0;
    float frictionCoe = 0.1;
    float gravity = 100;

    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics yay");
    window.setFramerateLimit(60);

    float boundRight = (float)window.getSize().x;
    float boundLeft = 0;
    float boundBottom = 550;
    float boundTop = 50;

    float e = 0.6;

    // set the scene
    Player groundS = {"groundS", 400, 550 + 25, 2e9, "rectangle", 800, 50};
    Player skyS = {"skyS", 400, 25, 2e9, "rectangle", 800, 50};
    Player left = {"left", -25, 300, 2e9, "rectangle", 50, 496};
    Player right = {"right", 825, 300, 2e9, "rectangle", 50, 496};

    Player sBob = {"sBob", 50, 100, 2, "triangle", 64};
    Player sPat = {"sPat", 400, 100, 2, "triangle", 64};

    Player sBox = {"sBox", 200, 300, 4, "rectangle", 200, 50};

    std::vector<Player*> players = {&sBob, &sPat, &sBox, &groundS, &skyS, &left, &right};
    
    sBob.applyForce(-3*Pi/2, gravity);
    sPat.applyForce(-3*Pi/2, gravity);
    sBox.applyForce(-3*Pi/2, gravity);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        window.clear(sf::Color(180, 255, 180));

        // -------------------
        frame++;

        if ((!delay || (frame - delay) > 10))
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
            { 
                sBob.pulse(Pi, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
            { 
                sBob.pulse(0, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
            { 
                sBob.pulse(-Pi/2, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
            { 
                sBob.pulse(-3*Pi/2, 100);
                delay = frame;
            }
        }

        {
            for (auto p : players)
                p->updatePos();

            for (auto p1 : players)
                for (auto p2 : players)
                    if (p1 != p2)
                        Physics::checkOverlap(*p1, *p2, e);
        }

        sBob.draw(window);
        sPat.draw(window);
        sBox.draw(window);

        groundS.draw(window, sf::Color(196, 164, 132));
        skyS.draw(window, sf::Color(135, 206, 235));
        left.draw(window);
        right.draw(window);

        // -------------------

        window.display();
    }
}