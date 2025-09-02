import { MenuItem } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";

interface ButtonDeleteProps {
  onClick: () => void;
  description?: string;
}

const ButtonDelete = ({ onClick, description }: ButtonDeleteProps) => {
  return (
    <MenuItem title={description} onClick={onClick}>
      <DeleteIcon fontSize="small" style={{ marginRight: 2 }} color="error"  />
      {/*Eliminar*/}
    </MenuItem>
  );
};

export default ButtonDelete;