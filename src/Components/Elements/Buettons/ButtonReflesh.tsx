import { MenuItem } from "@mui/material";
import RefreshIcon from '@mui/icons-material/Refresh';

interface ButtonRefleshProps {
  onClick: () => void;
  description?: string;
}

const ButtonReflesh = ({ onClick, description  }: ButtonRefleshProps) => {
  return (
    <MenuItem title={description} onClick={onClick}
      sx={{
        minHeight: "28px",  // baja la altura mínima
        padding: "2px 1px", // menos espacio interno
        fontSize: "0.8rem", // texto más chico (si hubiera texto)
      }}
    >
      <RefreshIcon fontSize="small" style={{ marginRight: 2 }} color="primary"/>
    </MenuItem>
  );
};

export default ButtonReflesh;