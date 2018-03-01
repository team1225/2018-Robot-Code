set tabstop=4
set softtabstop=4
set shiftwidth=4
set noexpandtab

set colorcolumn=110
highlight ColorColumn ctermbg=darkgray

let &path.="src/,src/Subsystems/,~/wpilib/cpp/current/include/,/usr/arm-frc-linux-gnueabi/,"

"nnoremap <C-b> :w <CR> :!make clean && make <CR>
nnoremap <C-b> :make <CR>
nnoremap <C-d> :!ant <CR>

