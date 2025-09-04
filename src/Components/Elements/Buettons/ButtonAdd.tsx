import { MenuItem } from "@mui/material";
import AddIcon from '@mui/icons-material/Add';

interface ButtonAddProps {
  onClick: () => void;
  description?: string;
}

const ButtonAdd = ({ onClick, description }: ButtonAddProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
      minHeight: "28px",  // baja la altura mínima
      padding: "2px 1px", // menos espacio interno
      fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <AddIcon fontSize="small" style={{ marginRight: 2 }} color="info"/>
    </MenuItem>
  );
};

export default ButtonAdd;