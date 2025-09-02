import React, { useState } from "react";
import { IconButton, Menu, MenuItem, ListItemIcon, ListItemText } from "@mui/material";
import MenuIcon from "@mui/icons-material/Menu"; // hamburger menu
import AddIcon from "@mui/icons-material/Add";
import EditIcon from "@mui/icons-material/Edit";
import DeleteIcon from "@mui/icons-material/Delete";

type Props = {
  onAdd?: () => void;
  onEdit?: () => void;
  onDelete?: () => void;
};

const MenuRutina: React.FC<Props> = ({ onAdd, onEdit, onDelete }) => {
  const [anchorEl, setAnchorEl] = useState<null | HTMLElement>(null);
  const open = Boolean(anchorEl);

  const handleOpen = (event: React.MouseEvent<HTMLElement>) => {
    setAnchorEl(event.currentTarget);
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  return (
    <>
      {/* Botón principal con ícono hamburger */}
      <IconButton
        onClick={handleOpen}
        size="small"
        sx={{ "&:focus": { outline: "none" } }}
      >
        <MenuIcon />
      </IconButton>

      {/* Menú desplegable */}
      <Menu
        anchorEl={anchorEl}
        open={open}
        onClose={handleClose}
        anchorOrigin={{
          vertical: "bottom",
          horizontal: "right",
        }}
        transformOrigin={{
          vertical: "top",
          horizontal: "right",
        }}
      >
        <MenuItem
          onClick={() => {
            handleClose();
            onAdd?.();
          }}
        >
          <ListItemIcon>
            <AddIcon fontSize="small" color="success" />
          </ListItemIcon>
          <ListItemText primary="Agregar" />
        </MenuItem>

        <MenuItem
          onClick={() => {
            handleClose();
            onEdit?.();
          }}
        >
          <ListItemIcon>
            <EditIcon fontSize="small" color="primary" />
          </ListItemIcon>
          <ListItemText primary="Editar" />
        </MenuItem>

        <MenuItem
          onClick={() => {
            handleClose();
            onDelete?.();
          }}
        >
          <ListItemIcon>
            <DeleteIcon fontSize="small" color="error" />
          </ListItemIcon>
          <ListItemText primary="Eliminar" />
        </MenuItem>
      </Menu>
    </>
  );
};

export default MenuRutina;
