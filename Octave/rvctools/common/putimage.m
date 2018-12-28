function [ fpath ] = putimage(h, movie,  i, fmt, iW, iH)
%% Saves image associated with given figure handle 'h'and frame index
%% 'i' in format specified by 'fmt' and size iW x iH. Returns path to
%% saved image file.

    fpath = sprintf ("%s/%ss/%04d.%s", movie, fmt, i, fmt);
    psiz = sprintf("-S%d,%d", iW, iH);
    print(h, fpath, psiz);
  
endfunction
