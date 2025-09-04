import { MenuItem } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";

interface ButtonDeleteProps {
  onClick: () => void;
  description?: string;
}

const ButtonDelete = ({ onClick, description }: ButtonDeleteProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
        minHeight: "28px",  // baja la altura mínima
        padding: "2px 1px", // menos espacio interno
        fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <DeleteIcon fontSize="small" style={{ marginRight: 2 }} color="error"  />
      {/*Eliminar*/}
    </MenuItem>
  );
};

export default ButtonDelete;