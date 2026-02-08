import { MenuItem } from "@mui/material";
import EditIcon from "@mui/icons-material/Edit";

interface ButtonEditProps {
  onClick: () => void;
  description?: string;
}

const ButtonEdit = ({ onClick, description }: ButtonEditProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
        minHeight: "28px",  // baja la altura mínima
        padding: "2px 1px", // menos espacio interno
        fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <EditIcon fontSize="small" style={{ marginRight: 2 }} />
      {/*Editar*/}
    </MenuItem>
  );
};

export default ButtonEdit;