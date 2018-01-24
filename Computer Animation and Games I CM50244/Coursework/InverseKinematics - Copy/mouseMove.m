function [x, y] = mouseMove(hobject,eventdata)
        L = get(gca,'currentpoint');
        x = L(1);
        y = L(2);
    end