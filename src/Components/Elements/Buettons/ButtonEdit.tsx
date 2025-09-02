import { MenuItem } from "@mui/material";
import EditIcon from "@mui/icons-material/Edit";

interface ButtonEditProps {
  onClick: () => void;
}

const ButtonEdit = ({ onClick }: ButtonEditProps) => {
  return (
    <MenuItem onClick={onClick}>
      <EditIcon fontSize="small" style={{ marginRight: 2 }} />
      {/*Editar*/}
    </MenuItem>
  );
};

export default ButtonEdit;